#!/bin/sh
set -u

BIN=${1:-/userdata/robot_slamd/robot_slamd}
CFG=${2:-/userdata/robot_slamd/config/config.board_sim_spin_scan_smoke.yaml}
OUT=${3:-/userdata/robot_slamd_board_runtime_profile_smoke}
DUR=${4:-20}
PROF=/tmp/runtime_profile.csv

rm -rf "$OUT"
mkdir -p "$OUT"

"$BIN" --config "$CFG" --duration-s "$DUR" --output-dir "$OUT" &
PID=$!

echo "timestamp_s,elapsed_s,pid,cpu_percent,rss_kb,vms_kb,thread_count" > "$PROF"
NCPU=$(grep -c "^processor" /proc/cpuinfo 2>/dev/null || echo 1)
[ -n "$NCPU" ] || NCPU=1
[ "$NCPU" -gt 0 ] || NCPU=1

cpu_total() {
    awk '/^cpu /{s=0; for(i=2;i<=NF;i++)s+=$i; print s}' /proc/stat
}

proc_ticks() {
    [ -r "/proc/$PID/stat" ] || return 1
    awk '{print $14+$15}' "/proc/$PID/stat" 2>/dev/null
}

status_value() {
    key=$1
    awk -v key="$key" '$1 == key ":" {print $2}' "/proc/$PID/status" 2>/dev/null
}

CPU0=$(cpu_total)
PROC0=$(proc_ticks)
START=$(date +%s)

while kill -0 "$PID" 2>/dev/null; do
    sleep 1
    kill -0 "$PID" 2>/dev/null || break
    NOW=$(date +%s)
    CPU1=$(cpu_total)
    PROC1=$(proc_ticks) || break
    DC=$((CPU1 - CPU0))
    DP=$((PROC1 - PROC0))
    PCT=$(awk -v dp="$DP" -v dc="$DC" -v n="$NCPU" 'BEGIN{if(dc>0)printf "%.3f",100.0*n*dp/dc; else printf "0.000"}')
    RSS=$(status_value VmRSS)
    VMS=$(status_value VmSize)
    THR=$(status_value Threads)
    [ -n "$RSS" ] || RSS=0
    [ -n "$VMS" ] || VMS=0
    [ -n "$THR" ] || THR=0
    EL=$((NOW - START))
    echo "$NOW,$EL,$PID,$PCT,$RSS,$VMS,$THR" >> "$PROF"
    CPU0=$CPU1
    PROC0=$PROC1
done

wait "$PID"
RC=$?
cp "$PROF" "$OUT/latest/runtime_profile.csv"
exit "$RC"
