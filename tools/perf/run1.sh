#!/bin/bash
# usage: run1.sh <sw|gpu> <scenario> <label> [VAR=val ...]   (build via $BIN, default build/Navigator)
S=${PERF_SCRATCH:?set PERF_SCRATCH to a scratch dir that contains perfdeep/ and sdk021/}
D=$S/perfdeep
mode=$1; scen=$2; label=$3; shift 3
BIN=${BIN:-$D/build/Navigator}
T=$(mktemp -d -p $D)
out=$D/runs/${label}.jsonl
export VAYU_VFS_DIR=$T/vfs; mkdir -p $VAYU_VFS_DIR
export VSIM_FIFO_SUFFIX=_perf$$
export VAYU_SITL_MODULE=$S/sdk021/vayu-sitl-sdk/lib/libvayu_sitl.so
export XDG_CONFIG_HOME=$T/cfg XDG_DATA_HOME=$T/data XDG_CACHE_HOME=$T/cache
mkdir -p $XDG_CONFIG_HOME $XDG_DATA_HOME $XDG_CACHE_HOME
cat /proc/loadavg > $D/runs/${label}.load
export QT_QPA_PLATFORM=xcb; unset WAYLAND_DISPLAY
export NAVIGATOR_PROFILE=$scen NAVIGATOR_PROFILE_OUT=$out NAVIGATOR_PROFILE_TAG=$mode
for kv in "$@"; do export "$kv"; done
if [ "$mode" = sw ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  timeout 300 xvfb-run -a -s "-screen 0 1920x1080x24 +extension GLX" $BIN > $T/stdout.log 2>&1
else
  [ "$DISPLAY" = ":0" ] || { echo "gpu mode needs DISPLAY=:0 in env"; exit 2; }
  export DISPLAY=:0
  nvidia-smi pmon -c 1 > $D/runs/${label}.pre 2>&1; nvidia-smi --query-gpu=utilization.gpu,memory.used,power.draw,clocks.sm --format=csv,noheader >> $D/runs/${label}.pre 2>&1
  nvidia-smi --query-gpu=utilization.gpu,memory.used,power.draw,clocks.sm --format=csv,noheader -l 1 > $D/runs/${label}.smi 2>&1 &
  smi=$!
  timeout 300 $BIN > $T/stdout.log 2>&1
  kill $smi 2>/dev/null
fi
cat /proc/loadavg >> $D/runs/${label}.load
echo "exit=$? out=$out"; tail -3 $T/stdout.log
cp $T/stdout.log $D/runs/${label}.stdout.log
rm -rf $T
