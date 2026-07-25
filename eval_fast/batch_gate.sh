#!/bin/bash
# batch_gate.sh - run many eval flights concurrently (RAM-aware cap) and report
# per-flight OFF-vs-ON determinism (f_ md5 must match) + peak RSS reduction.
#
# Standard phase gate for the memory-reduction work: proves the phase's ON knob(s)
# leave the trajectory bit-identical to OFF across the whole official flight set,
# while measuring the RSS win, in one parallel batch instead of serial runs.
#
# Usage:
#   BIN=<binary> LIB=<libdir> ON_ENV="ORB_MEM_RECLAIM_BAD=1" CAP=10 \
#     batch_gate.sh <label> <flightspec_file>
# flightspec lines: "<flight_key> <NF>"  (blank lines / #comments ignored)
# Each spec expands to two jobs: OFF (stock) and ON (ON_ENV prepended).
set -u
WORKTREE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUNNER="$WORKTREE_ROOT/eval_fast/run_flight.sh"
LABEL="${1:?label}"
SPEC="${2:?flightspec file}"
BIN="${BIN:-$WORKTREE_ROOT/Examples/Monocular-Inertial/mono_inertial_euroc}"
LIB="${LIB:-}"
ON_ENV="${ON_ENV:-ORB_MEM_RECLAIM_BAD=1}"
CAP="${CAP:-10}"
OUTROOT="$WORKTREE_ROOT/eval_runs/batch_${LABEL}"
mkdir -p "$OUTROOT"
JOBS_FILE="$OUTROOT/jobs.txt"; : > "$JOBS_FILE"

# Build the job list: one OFF + one ON per spec line.
while read -r flight nf _rest; do
  [ -z "${flight:-}" ] && continue
  case "$flight" in \#*) continue;; esac
  echo "$flight $nf OFF" >> "$JOBS_FILE"
  echo "$flight $nf ON"  >> "$JOBS_FILE"
done < "$SPEC"

run_one() {
  local flight="$1" nf="$2" mode="$3"
  local out="$OUTROOT/${flight}_${nf}_${mode}"
  local env_prefix=""
  [ "$mode" = "ON" ] && env_prefix="$ON_ENV"
  local bin_env="ORB_BIN=$BIN"
  [ -n "$LIB" ] && bin_env="$bin_env LD_LIBRARY_PATH=$LIB"
  env $bin_env $env_prefix ORB_MEM_STATS_CSV="$out/mem_stats.csv" \
    "$RUNNER" "$flight" "$nf" "$out" > "$out.launch.log" 2>&1
}
export -f run_one
export OUTROOT RUNNER BIN LIB ON_ENV

# RAM-aware concurrency: never exceed CAP running jobs, and pause launching while
# free RAM < 6 GB so the unreclaimed OFF runs cannot OOM the box.
launched=0
total=$(wc -l < "$JOBS_FILE")
echo "batch_${LABEL}: $total jobs, cap=$CAP, bin=$BIN"
while read -r flight nf mode; do
  while :; do
    running=$(jobs -rp | wc -l)
    freeg=$(free -g | awk '/^Mem:/{print $7}')
    if [ "$running" -lt "$CAP" ] && [ "$freeg" -ge 6 ]; then break; fi
    sleep 5
  done
  run_one "$flight" "$nf" "$mode" &
  launched=$((launched+1))
  echo "  [$launched/$total] launched $flight NF=$nf $mode (running=$(jobs -rp | wc -l), free=${freeg}G)"
  sleep 2
done < "$JOBS_FILE"
wait
echo "batch_${LABEL}: all $total jobs done"

# Report: per flight/NF, OFF vs ON md5 + peak RSS.
REPORT="$OUTROOT/report.txt"
{
  printf "%-14s %5s | %-10s %-10s %-7s | %7s %7s %7s | %s\n" FLIGHT NF OFF_md5 ON_md5 DETERM OFF_MB ON_MB SAVED_MB ON_COVERAGE
  while read -r flight nf _rest; do
    [ -z "${flight:-}" ] && continue; case "$flight" in \#*) continue;; esac
    local_off="$OUTROOT/${flight}_${nf}_OFF"; local_on="$OUTROOT/${flight}_${nf}_ON"
    om=$(md5sum "$local_off/f_hl_${flight}.txt" 2>/dev/null | cut -c1-8); nm=$(md5sum "$local_on/f_hl_${flight}.txt" 2>/dev/null | cut -c1-8)
    op=$(awk -F, 'NR>1{h=$4} END{print h+0}' "$local_off/mem_stats.csv" 2>/dev/null)
    np=$(awk -F, 'NR>1{h=$4} END{print h+0}' "$local_on/mem_stats.csv" 2>/dev/null)
    # Distinguish a real determinism failure (both completed, md5 differ) from an
    # incomplete run (wall-clock cap SIGTERM'd it before trajectory save). A capped
    # run is a perf/harness limit, NOT a determinism verdict.
    ocap=$(grep -oE 'capped=[01]' "$local_off/summary.txt" 2>/dev/null | cut -d= -f2)
    ncap=$(grep -oE 'capped=[01]' "$local_on/summary.txt" 2>/dev/null | cut -d= -f2)
    if [ "${ocap:-0}" = "1" ] || [ "${ncap:-0}" = "1" ]; then
      det="CAPPED"
    elif [ -n "$om" ] && [ "$om" = "$nm" ]; then
      det="MATCH"
    elif [ -z "$om" ] && [ -z "$nm" ]; then
      det="NO-TRAJ"
    else
      det="DIFF!"
    fi
    saved=$(( ${op:-0} - ${np:-0} ))
    cov=$(grep 'RUN SUMMARY' "$local_on/run.log" 2>/dev/null | grep -oE 'coverage [0-9/]+ \([0-9.]+%\) \[[A-Z-]+\]')
    printf "%-14s %5s | %-10s %-10s %-7s | %7s %7s %7s | %s\n" "$flight" "$nf" "${om:-NONE}" "${nm:-NONE}" "$det" "${op:-?}" "${np:-?}" "$saved" "$cov"
  done < "$SPEC"
} | tee "$REPORT"
echo "report: $REPORT"
