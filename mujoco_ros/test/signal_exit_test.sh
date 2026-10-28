#!/usr/bin/env bash
# Start the unpaced mujoco_node, send a signal and require a clean exit within a deadline.
# usage: signal_exit_test.sh <mujoco_node> <modelfile> <INT|TERM> [deadline_s]
set -u
NODE=$1
MODEL=$2
SIG=$3
DEADLINE=${4:-10}

"$NODE" --ros-args -p headless:=true -p no_render:=true -p unpause:=true -p realtime:=-1.0 \
	-p modelfile:="$MODEL" >/dev/null 2>&1 &
PID=$!
sleep 4
if ! kill -0 "$PID" 2>/dev/null; then
	echo "FAIL: mujoco_node died before the signal was sent"
	exit 1
fi

kill -"$SIG" "$PID"
for _ in $(seq 1 "$DEADLINE"); do
	if ! kill -0 "$PID" 2>/dev/null; then
		wait "$PID"
		echo "PASS: SIG$SIG exit"
		exit 0
	fi
	sleep 1
done

echo "FAIL: mujoco_node ignored SIG$SIG for ${DEADLINE}s"
kill -9 "$PID" 2>/dev/null
exit 1
