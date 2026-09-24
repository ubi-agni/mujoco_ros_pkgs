#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
usage: run_render_performance_interleaved_bccb.sh \
  --baseline-exec PATH --candidate-exec PATH \
  --camera-count N \
  --output-dir DIR \
  [--backend EGL|OSMESA] [--blocks N] [--iterations N] [--date-tag YYYYMMDD] \
  [--candidate-only]

Runs balanced B-C-C-B interleaved trials (default 6 blocks => 12 trials per revision).
With --candidate-only, skips baseline and runs C-C per block (12 candidate trials).
EOF
}

BASELINE_EXEC=""
CANDIDATE_EXEC=""
CAMERA_COUNT=""
OUTPUT_DIR=""
BACKEND="EGL"
BLOCKS=6
ITERATIONS=1000
DATE_TAG="$(date +%Y%m%d)"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
CPU_AFFINITY="${CPU_AFFINITY:-0-15}"
CANDIDATE_ONLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --baseline-exec) BASELINE_EXEC="$2"; shift 2 ;;
    --candidate-exec) CANDIDATE_EXEC="$2"; shift 2 ;;
    --camera-count) CAMERA_COUNT="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;
    --blocks) BLOCKS="$2"; shift 2 ;;
    --iterations) ITERATIONS="$2"; shift 2 ;;
    --date-tag) DATE_TAG="$2"; shift 2 ;;
    --candidate-only) CANDIDATE_ONLY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${CANDIDATE_EXEC}" || -z "${CAMERA_COUNT}" || -z "${OUTPUT_DIR}" ]]; then
  usage >&2
  exit 2
fi
if [[ "${CANDIDATE_ONLY}" -eq 0 && -z "${BASELINE_EXEC}" ]]; then
  usage >&2
  exit 2
fi

if [[ "${BACKEND}" != "EGL" && "${BACKEND}" != "OSMESA" ]]; then
  echo "unsupported --backend '${BACKEND}'; expected EGL or OSMESA" >&2
  usage >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MERGE_SCRIPT="${SCRIPT_DIR}/merge_render_performance_trials.py"

mkdir -p "${OUTPUT_DIR}"

# Concurrent MuJoCo/ROS nodes publish /clock and emit "time reset happened".
# Refuse to start while any other bench or mujoco_ros test process is alive.
assert_no_competing_mujoco_ros() {
  python3 - <<'PY'
import os, pathlib, sys

# Match real binaries / scripts only — not shells whose argv quotes these names.
binary_names = {
    "mujoco_render_test",
    "render_performance_test",
    "mujoco_server",
}
script_names = {
    "python_bindings_retirement_only.py",
}

def walk_ancestors(pid: int) -> set[int]:
    seen = set()
    while pid > 1 and pid not in seen:
        seen.add(pid)
        try:
            status = pathlib.Path(f"/proc/{pid}/status").read_text()
            pid = int(status.split("PPid:")[1].split()[0])
        except Exception:
            break
    return seen

ignore = walk_ancestors(os.getpid()) | walk_ancestors(os.getppid())
hits = []
for pid_dir in pathlib.Path("/proc").iterdir():
    if not pid_dir.name.isdigit():
        continue
    pid = int(pid_dir.name)
    if pid in ignore:
        continue
    try:
        raw = (pid_dir / "cmdline").read_bytes()
    except Exception:
        continue
    if not raw:
        continue
    args = [a.decode("utf-8", "replace") for a in raw.split(b"\0") if a]
    if not args:
        continue
    argv0 = pathlib.Path(args[0]).name
    if argv0 in {"bash", "sh", "dash", "zsh", "fish"}:
        # Only count if an explicit script path is argv1+.
        if not any(pathlib.Path(a).name in script_names for a in args[1:]):
            continue
    if argv0 in binary_names or any(pathlib.Path(a).name in script_names for a in args):
        hits.append(f"{pid} {' '.join(args)[:200]}")
        continue
    try:
        exe_name = (pid_dir / "exe").resolve().name
    except Exception:
        exe_name = ""
    if exe_name in binary_names:
        hits.append(f"{pid} exe={exe_name} {' '.join(args)[:160]}")
if hits:
    print("REFUSING: competing MuJoCo/ROS/bench processes still running:", file=sys.stderr)
    print("\n".join(hits), file=sys.stderr)
    print("Kill them, then re-run camera counts one at a time.", file=sys.stderr)
    raise SystemExit(3)
PY
}
assert_no_competing_mujoco_ros
# Between camera-count campaigns the caller must also wait until this script exits.

if command -v cpupower >/dev/null 2>&1; then
  sudo -n cpupower frequency-set -g performance >/dev/null 2>&1 || true
fi

record_gpu_state() {
  local label="$1"
  local path="$2"
  {
    echo "{\"label\": \"${label}\", \"timestamp\": \"$(date -Iseconds)\"}"
    if command -v nvidia-smi >/dev/null 2>&1; then
      nvidia-smi --query-gpu=name,driver_version,temperature.gpu,clocks.sm,clocks.max.sm,power.draw,power.limit,pstate,utilization.gpu \
        --format=csv,noheader,nounits || true
    fi
  } > "${path}"
}

run_trial() {
  local revision="$1"
  local letter="$2"
  local block_index="$3"
  local trial_index="$4"
  local executable="$5"
  local output_json="$6"

  # Executable lives at <ws>/build/mujoco_ros/test/render_performance_test.
  # AMENT needs <ws>/install on AMENT_PREFIX_PATH or package share lookup fails.
  local ws_root
  ws_root="$(cd "$(dirname "${executable}")/../../.." && pwd)"
  local setup_bash="${ws_root}/install/setup.bash"
  if [[ ! -f "${setup_bash}" ]]; then
    echo "missing install overlay for ${executable}: ${setup_bash}" >&2
    return 1
  fi

  export ROS_DOMAIN_ID
  local gpu_state="${output_json%.json}.gpu.json"
  record_gpu_state "${revision}-block${block_index}-trial${trial_index}-${letter}" "${gpu_state}"

  (
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
    # shellcheck disable=SC1091
    source "${setup_bash}"
    set -u
    export ROS_DOMAIN_ID
    taskset -c "${CPU_AFFINITY}" \
      "${executable}" \
      --backend "${BACKEND}" \
      --camera-count "${CAMERA_COUNT}" \
      --iterations "${ITERATIONS}" \
      --repeat 1 \
      --revision "${revision}" \
      --block-index "${block_index}" \
      --trial-index "${trial_index}" \
      --interleave-letter "${letter}" \
      --output "${output_json}"
  )
}

baseline_trials=()
candidate_trials=()
trial_index=0
if [[ "${CANDIDATE_ONLY}" -eq 1 ]]; then
  letters=(C C)
else
  letters=(B C C B)
fi

for block in $(seq 1 "${BLOCKS}"); do
  block_dir="${OUTPUT_DIR}/block-${block}"
  mkdir -p "${block_dir}"
  record_gpu_state "block-${block}-start" "${block_dir}/gpu-block-start.json"
  for letter in "${letters[@]}"; do
    trial_index=$((trial_index + 1))
    if [[ "${letter}" == "B" ]]; then
      revision="baseline"
      executable="${BASELINE_EXEC}"
    else
      revision="candidate"
      executable="${CANDIDATE_EXEC}"
    fi
    output_json="${block_dir}/trial-${trial_index}-${revision}.json"
    run_trial "${revision}" "${letter}" "${block}" "${trial_index}" \
      "${executable}" "${output_json}"
    if [[ "${revision}" == "baseline" ]]; then
      baseline_trials+=("${output_json}")
    else
      candidate_trials+=("${output_json}")
    fi
  done
  record_gpu_state "block-${block}-end" "${block_dir}/gpu-block-end.json"
done

candidate_merged="${OUTPUT_DIR}/architectural-optimization-interleaved-candidate-cam${CAMERA_COUNT}-${DATE_TAG}.json"
python3 "${MERGE_SCRIPT}" "${candidate_trials[@]}" --output "${candidate_merged}"
echo "candidate_merged=${candidate_merged}"

if [[ "${CANDIDATE_ONLY}" -eq 0 ]]; then
  baseline_merged="${OUTPUT_DIR}/architectural-optimization-interleaved-baseline-cam${CAMERA_COUNT}-${DATE_TAG}.json"
  python3 "${MERGE_SCRIPT}" "${baseline_trials[@]}" --output "${baseline_merged}"
  echo "baseline_merged=${baseline_merged}"
fi
