#!/usr/bin/env bash
# Advisory CI orchestration for OSMesa interleaved render performance campaigns.
set -euo pipefail

usage() {
  cat <<'EOF'
usage: run_render_performance_advisory_ci.sh \
  --repo-root PATH \
  --baseline-sha SHA \
  --candidate-sha SHA \
  --artifacts-dir PATH

Builds separate OSMesa Release workspaces for baseline and candidate revisions,
runs the B-C-C-B harness for camera counts 1/2/4, and emits normalized reports.
EOF
}

REPO_ROOT=""
BASELINE_SHA=""
CANDIDATE_SHA=""
ARTIFACTS_DIR=""
ROS_DISTRO="${ROS_DISTRO:-humble}"
MUJOCO_DIR="${MUJOCO_DIR:-/root/mujoco/3.3.5}"
CAMERA_COUNTS=(1 2 4)
ITERATIONS=1000
# Fast DDS rejects domain IDs above 232 (see docs/guardrails.md).
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-120}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root) REPO_ROOT="$2"; shift 2 ;;
    --baseline-sha) BASELINE_SHA="$2"; shift 2 ;;
    --candidate-sha) CANDIDATE_SHA="$2"; shift 2 ;;
    --artifacts-dir) ARTIFACTS_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${REPO_ROOT}" || -z "${BASELINE_SHA}" || -z "${CANDIDATE_SHA}" || -z "${ARTIFACTS_DIR}" ]]; then
  usage >&2
  exit 2
fi

# Container jobs often see checkout owned by a different uid than the runner user.
# Without this, `git archive` aborts with "detected dubious ownership".
git config --global --add safe.directory "${REPO_ROOT}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HARNESS="${SCRIPT_DIR}/run_render_performance_interleaved_bccb.sh"
REPORT_SCRIPT="${SCRIPT_DIR}/render_performance_report.py"

mkdir -p "${ARTIFACTS_DIR}/logs" "${ARTIFACTS_DIR}/raw" "${ARTIFACTS_DIR}/reports"

# under `set -u`, sourcing ROS setup fails on unset AMENT_TRACE_SETUP_FILES (and friends)
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

build_workspace() {
  local label="$1"
  local source_root="$2"
  local ws_root="${ARTIFACTS_DIR}/workspaces/${label}"
  local build_log="${ARTIFACTS_DIR}/logs/build-${label}.log"
  mkdir -p "${ws_root}/src"
  ln -sfn "${source_root}" "${ws_root}/src/mujoco_ros_pkgs"
  # Stream build output to the log and stderr only — stdout is reserved for ws_root
  # so callers can safely capture `$(build_workspace ...)`.
  if ! (
    cd "${ws_root}"
    export MUJOCO_DIR
    # Prefer the workflow-provided cache dir; fall back for local runs.
    export CCACHE_DIR="${CCACHE_DIR:-${HOME}/.ccache}"
    export CCACHE_NOHASHDIR="${CCACHE_NOHASHDIR:-1}"
    mkdir -p "${CCACHE_DIR}"
    # Relative basedir so baseline/candidate workspaces can share hits.
    export CCACHE_BASEDIR="${ws_root}"
    colcon build --packages-select mujoco_ros_msgs mujoco_ros_testing_utils mujoco_ros \
      --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
      -DWITH_GUI=OFF -DOFFSCREEN_BACKEND=OSMESA -DMUJOCO_DIR="${MUJOCO_DIR}" \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      2>&1 | tee "${build_log}" >&2
  ); then
    echo "colcon build failed for ${label} (see ${build_log})" >&2
    exit 1
  fi
  printf '%s\n' "${ws_root}"
}

install_rosdeps() {
  local rosdep_log="${ARTIFACTS_DIR}/logs/rosdep-install.log"
  # MuJoCo comes from MUJOCO_DIR in the CI image, not apt/rosdep.
  {
    apt-get update -qq
    rosdep update --rosdistro "${ROS_DISTRO}" || true
    rosdep install --from-paths "$@" \
      --ignore-src -y --rosdistro "${ROS_DISTRO}" \
      --skip-keys mujoco
  } 2>&1 | tee "${rosdep_log}" >&2
}

assert_no_competing_processes() {
  python3 - <<'PY'
import os, pathlib, sys

binary_names = {"mujoco_render_test", "render_performance_test", "mujoco_server"}
script_names = {"python_bindings_retirement_only.py"}

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
        if not any(pathlib.Path(a).name in script_names for a in args[1:]):
            continue
    if argv0 in binary_names or any(pathlib.Path(a).name in script_names for a in args):
        hits.append(f"{pid} {' '.join(args)[:200]}")
if hits:
    print("REFUSING: competing MuJoCo/ROS/bench processes still running:", file=sys.stderr)
    print("\n".join(hits), file=sys.stderr)
    raise SystemExit(3)
PY
}

prepare_source_tree() {
  local sha="$1"
  local dest="$2"
  rm -rf "${dest}"
  mkdir -p "${dest}"
  git -C "${REPO_ROOT}" archive "${sha}" | tar -x -C "${dest}"
}

prepare_source_tree "${CANDIDATE_SHA}" "${ARTIFACTS_DIR}/source/candidate"
prepare_source_tree "${BASELINE_SHA}" "${ARTIFACTS_DIR}/source/baseline"

BASELINE_CMAKE="${ARTIFACTS_DIR}/source/baseline/mujoco_ros/test/CMakeLists.txt"
BASELINE_HAS_HARNESS=1
if [[ ! -f "${BASELINE_CMAKE}" ]] || ! grep -q 'add_executable(render_performance_test' "${BASELINE_CMAKE}"; then
  BASELINE_HAS_HARNESS=0
  echo "baseline ${BASELINE_SHA} lacks render_performance_test; running candidate-only campaign" >&2
fi

if [[ "${BASELINE_HAS_HARNESS}" -eq 1 ]]; then
  install_rosdeps \
    "${ARTIFACTS_DIR}/source/candidate" \
    "${ARTIFACTS_DIR}/source/baseline"
else
  install_rosdeps \
    "${ARTIFACTS_DIR}/source/candidate"
fi

CANDIDATE_WS="$(build_workspace candidate "${ARTIFACTS_DIR}/source/candidate")"
CANDIDATE_EXEC="${CANDIDATE_WS}/build/mujoco_ros/test/render_performance_test"
if [[ ! -x "${CANDIDATE_EXEC}" ]]; then
  echo "missing benchmark executable: ${CANDIDATE_EXEC}" >&2
  exit 1
fi

BASELINE_EXEC=""
if [[ "${BASELINE_HAS_HARNESS}" -eq 1 ]]; then
  BASELINE_WS="$(build_workspace baseline "${ARTIFACTS_DIR}/source/baseline")"
  BASELINE_EXEC="${BASELINE_WS}/build/mujoco_ros/test/render_performance_test"
  if [[ ! -x "${BASELINE_EXEC}" ]]; then
    echo "missing benchmark executable: ${BASELINE_EXEC}" >&2
    exit 1
  fi
fi

DATE_TAG="$(date +%Y%m%d)"
declare -a REPORT_PATHS=()

if (( BASE_ROS_DOMAIN_ID + ${#CAMERA_COUNTS[@]} - 1 > 232 )); then
  echo "BASE_ROS_DOMAIN_ID=${BASE_ROS_DOMAIN_ID} leaves camera domains above Fast DDS max 232" >&2
  exit 2
fi

domain_id="${BASE_ROS_DOMAIN_ID}"
for camera_count in "${CAMERA_COUNTS[@]}"; do
  assert_no_competing_processes
  output_dir="${ARTIFACTS_DIR}/raw/cam${camera_count}"
  rm -rf "${output_dir}"
  mkdir -p "${output_dir}"
  log_path="${ARTIFACTS_DIR}/logs/campaign-cam${camera_count}.log"
  harness_args=(
    --candidate-exec "${CANDIDATE_EXEC}"
    --camera-count "${camera_count}"
    --output-dir "${output_dir}"
    --backend OSMESA
    --iterations "${ITERATIONS}"
    --date-tag "${DATE_TAG}"
  )
  if [[ "${BASELINE_HAS_HARNESS}" -eq 1 ]]; then
    harness_args+=(--baseline-exec "${BASELINE_EXEC}")
  else
    harness_args+=(--candidate-only)
  fi
  if ! env ROS_DOMAIN_ID="${domain_id}" \
    "${HARNESS}" "${harness_args[@]}" \
    2>&1 | tee "${log_path}"; then
    echo "harness failed for camera_count=${camera_count}" >&2
    exit 1
  fi

  candidate_merged="${output_dir}/architectural-optimization-interleaved-candidate-cam${camera_count}-${DATE_TAG}.json"
  if [[ ! -f "${candidate_merged}" ]]; then
    echo "missing merged candidate receipt for camera_count=${camera_count}" >&2
    exit 1
  fi

  report_json="${ARTIFACTS_DIR}/reports/cam${camera_count}.json"
  report_md="${ARTIFACTS_DIR}/reports/cam${camera_count}.md"
  if [[ "${BASELINE_HAS_HARNESS}" -eq 1 ]]; then
    baseline_merged="${output_dir}/architectural-optimization-interleaved-baseline-cam${camera_count}-${DATE_TAG}.json"
    if [[ ! -f "${baseline_merged}" ]]; then
      echo "missing merged baseline receipt for camera_count=${camera_count}" >&2
      exit 1
    fi
    if ! python3 "${REPORT_SCRIPT}" compare \
      --baseline "${baseline_merged}" \
      --candidate "${candidate_merged}" \
      --baseline-ref "${BASELINE_SHA}" \
      --candidate-ref "${CANDIDATE_SHA}" \
      --json-out "${report_json}" \
      --markdown-out "${report_md}" \
      --emit-github-warnings; then
      echo "malformed or invalid receipt for camera_count=${camera_count}" >&2
      exit 1
    fi
  else
    if ! python3 "${REPORT_SCRIPT}" candidate-only \
      --candidate "${candidate_merged}" \
      --baseline-ref "${BASELINE_SHA}" \
      --candidate-ref "${CANDIDATE_SHA}" \
      --reason "baseline revision does not build render_performance_test" \
      --json-out "${report_json}" \
      --markdown-out "${report_md}"; then
      echo "malformed or invalid candidate receipt for camera_count=${camera_count}" >&2
      exit 1
    fi
  fi
  REPORT_PATHS+=("${report_json}")
  domain_id=$((domain_id + 1))
done

SUMMARY_JSON="${ARTIFACTS_DIR}/reports/summary.json"
SUMMARY_MD="${ARTIFACTS_DIR}/reports/summary.md"
python3 "${REPORT_SCRIPT}" aggregate "${REPORT_PATHS[@]}" \
  --json-out "${SUMMARY_JSON}" \
  --markdown-out "${SUMMARY_MD}" \
  --emit-github-warnings

echo "summary_markdown=${SUMMARY_MD}"
if [[ "${BASELINE_HAS_HARNESS}" -eq 1 ]]; then
  echo "advisory campaign complete"
else
  echo "advisory candidate-only campaign complete"
fi
