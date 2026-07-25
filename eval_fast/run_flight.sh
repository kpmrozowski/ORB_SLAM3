#!/bin/bash
# Headless mono-inertial memory/CPU baseline run for one flight: deterministic campaign env
# (no viewer/GPS overlay), optional ORBextractor.nFeatures override, RSS/CPU sampler.
# Generalized from the session scratchpad's headless_212.sh runner (same env block), with one
# bug fixed: that script wrapped the binary in `timeout 3600` and sampled `$!`, which is the
# timeout process's PID, not the binary's - every RSS/CPU sample was garbage for the whole run.
# Here the binary runs directly with `&` and the sampler tracks its real PID; the 3600s
# `timeout` is replaced with a 4000s cap enforced inside the sampler loop itself.
#
# Usage: eval_fast/run_flight.sh <flight:182_golem27|212_golem27> <nf:2500|5000> <out_dir> [times_file]
#   flight     - resolves dataset dir /home/kmro/praca/dev/orbslam3-eval/dataset/hist_<flight>
#   nf         - ORBextractor.nFeatures override, applied via sed to a per-run config copy
#                (the canonical eval_fast/config_<flight>.yaml stays untouched for provenance)
#   out_dir    - destination for f_/kf_ trajectories, run.log, monitor.csv, summary.txt,
#                config.yaml (the per-run sed'd config), frame_stats.csv
#   times_file - optional cam0_times.txt override (default: the flight's full cam0_times.txt);
#                pass eval_fast/times_212_3min.txt for the ~3min fast md5 gate.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKTREE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROOT_EVAL=/home/kmro/praca/dev/orbslam3-eval

export LD_LIBRARY_PATH="/home/kmro/praca/dev/dmvio-loop/third_party/local/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"

FLIGHT="${1:?usage: run_flight.sh <182_golem27|212_golem27> <nf> <out_dir> [times_file]}"
NF="${2:?usage: run_flight.sh <182_golem27|212_golem27> <nf> <out_dir> [times_file]}"
OUT_DIR="${3:?usage: run_flight.sh <182_golem27|212_golem27> <nf> <out_dir> [times_file]}"

case "$FLIGHT" in
    *_*) ;;   # any <token>_<vehicle> flight key; dataset-dir existence (below) is the real guard
    *) echo "run_flight.sh: '$FLIGHT' is not a flight key (expected <token>_<vehicle>)" >&2; exit 1 ;;
esac
if ! [[ "$NF" =~ ^[0-9]+$ ]]; then
    echo "run_flight.sh: nf must be a positive integer (got '$NF')" >&2
    exit 1
fi

DS_DIR="$ROOT_EVAL/dataset/hist_$FLIGHT"
[ -d "$DS_DIR" ] || { echo "run_flight.sh: dataset dir not found: $DS_DIR" >&2; exit 1; }
TIMES_FILE="${4:-$DS_DIR/cam0_times.txt}"
[ -f "$TIMES_FILE" ] || { echo "run_flight.sh: times file not found: $TIMES_FILE" >&2; exit 1; }
TIMES_FILE="$(realpath "$TIMES_FILE")"   # absolute: the binary runs with cwd=OUT_DIR, and a relative
                                         # times path would make its ifstream.open fail there - which
                                         # the example's `while(!eof())` loader turns into a CPU spin.

BIN="${ORB_BIN:-$WORKTREE_ROOT/Examples/Monocular-Inertial/mono_inertial_euroc}"   # ORB_BIN overrides (A/B cross-build checks), same convention as run_viewer.sh:14
VOCAB="$WORKTREE_ROOT/Vocabulary/ORBvoc.txt"
[ -x "$BIN" ] || { echo "run_flight.sh: binary not built: $BIN (run ./build.sh)" >&2; exit 1; }
[ -f "$VOCAB" ] || { echo "run_flight.sh: vocabulary not found: $VOCAB (run ./build.sh)" >&2; exit 1; }

# --- Resolve/refresh the per-flight canonical base config (checked into eval_fast/) ----------------
# 212_golem27's canonical copy is the exact config used to record the W0 baseline (provenance
# matters for md5 equality) and cannot be regenerated - it must already exist.
# 182_golem27 is absent from flights.json, so it self-heals via the same resolution
# eval/ic_tune/run_viewer.sh:29-51 uses (flights.json first, else configs/hist/*_noup.yaml).
BASE_CONFIG="$SCRIPT_DIR/config_${FLIGHT}.yaml"
if [ ! -f "$BASE_CONFIG" ]; then
    case "$FLIGHT" in
        212_golem27)
            echo "run_flight.sh: missing canonical $BASE_CONFIG (W0 baseline provenance copy - restore from git, cannot regenerate)" >&2
            exit 1
            ;;
        *)
            RESOLVED=$(python3 -c "
import glob, json, os
token = '$FLIGHT'
root = '$ROOT_EVAL'
flights_json = os.path.join(root, 'eval_out', 'ic_tune', 'flights.json')
chosen = None
if os.path.isfile(flights_json):
    for flight in json.load(open(flights_json))['flights']:
        if token in flight['key']:
            chosen = flight['config']; break
if chosen is None:
    matches = sorted(glob.glob(os.path.join(root, 'configs', 'hist', token + '_*_noup.yaml')))
    if not matches:
        matches = sorted(glob.glob(os.path.join(root, 'configs', 'hist', token + '_*.yaml')))
    if len(matches) != 1:
        raise SystemExit(f'ambiguous/none configs/hist for {token!r}: {matches}')
    chosen = matches[0]
print(chosen)
") || exit 1
            cp "$RESOLVED" "$BASE_CONFIG"
            ;;
    esac
fi

mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"   # canonicalize: the binary is launched from inside OUT_DIR
                                    # below, so a relative OUT_DIR would otherwise be re-resolved
                                    # against the new cwd for run.log/summary.txt/monitor.csv.
RUN_CONFIG="$OUT_DIR/config.yaml"
cp "$BASE_CONFIG" "$RUN_CONFIG"
sed -i -E "s/^ORBextractor\.nFeatures:.*/ORBextractor.nFeatures: $NF/" "$RUN_CONFIG"
grep -q "^ORBextractor.nFeatures: $NF\$" "$RUN_CONFIG" || { echo "run_flight.sh: failed to set nFeatures in $RUN_CONFIG" >&2; exit 1; }

# --- Deterministic campaign env (no viewer/GPS overlay), matching headless_212.sh -------------------
export ORB_DETERMINISTIC=1
export OMP_NUM_THREADS=1
export ORB_CLAHE=3.0
export ORB_TSJUMP_S=3.0
[ -f "$DS_DIR/ref_baro.csv" ] && export ORB_BARO_CSV="$DS_DIR/ref_baro.csv"
export ORB_BARO_SIGMA=0.15
export ORB_BARO_GATE=6
export ORB_BARO_FRAME_SIGMA=0.5
[ -f "$DS_DIR/ref_mag.csv" ] && export ORB_MAG_CSV="$DS_DIR/ref_mag.csv"
export ORB_MAG_SIGMA_DEG=10
export ORB_DIVERGE_VMAX=30
export ORB_DIVERGE_COUNT=4
export ORB_STATS_CSV="$OUT_DIR/frame_stats.csv"

PREFIX="hl_${FLIGHT}"
CLK_TCK=$(getconf CLK_TCK)
INT=5
CAP_S=${CAP_S:-4000}   # wall-clock kill cap (s); override for big flights whose per-frame
                       # compute runs slower than real-time (e.g. 242_golem17 needs ~9000)

# Run the binary directly (no `timeout` wrapper - see header note) so $! is its real PID.
( cd "$OUT_DIR" && exec "$BIN" "$VOCAB" "$RUN_CONFIG" "$DS_DIR" "$TIMES_FILE" "$PREFIX" > "$OUT_DIR/run.log" 2>&1 ) &
PID=$!
START_EPOCH=$(date +%s)

MON="$OUT_DIR/monitor.csv"
echo "elapsed_s,cpu_percent,rss_mb,vmhwm_mb,threads" > "$MON"
prev_jiffies=0
elapsed=0
capped=0
while kill -0 "$PID" 2>/dev/null; do
    sleep "$INT"
    elapsed=$((elapsed + INT))
    if [ -r "/proc/$PID/stat" ]; then
        stat_line=$(cat "/proc/$PID/stat" 2>/dev/null) || break
        rest="${stat_line##*) }"
        utime=$(echo "$rest" | awk '{print $12}')
        stime=$(echo "$rest" | awk '{print $13}')
        jiffies=$((utime + stime))
        cpu=$(awk -v j="$jiffies" -v p="$prev_jiffies" -v c="$CLK_TCK" -v i="$INT" 'BEGIN{printf "%.1f", 100.0*(j-p)/c/i}')
        prev_jiffies=$jiffies
        rss=$(awk '/^VmRSS/{printf "%.1f", $2/1024}' "/proc/$PID/status" 2>/dev/null)
        hwm=$(awk '/^VmHWM/{printf "%.1f", $2/1024}' "/proc/$PID/status" 2>/dev/null)
        thr=$(awk '/^Threads/{print $2}' "/proc/$PID/status" 2>/dev/null)
        [ -n "$rss" ] && echo "$elapsed,$cpu,$rss,$hwm,$thr" >> "$MON"
    fi
    if [ "$elapsed" -ge "$CAP_S" ]; then
        echo "run_flight.sh: ${CAP_S}s cap exceeded, killing PID $PID" >&2
        kill -TERM "$PID" 2>/dev/null
        sleep 5
        kill -KILL "$PID" 2>/dev/null
        capped=1
        break
    fi
done
wait "$PID" 2>/dev/null
EXIT_CODE=$?
END_EPOCH=$(date +%s)
WALL_S=$((END_EPOCH - START_EPOCH))
# Average CPU over the whole run (total CPU jiffies / wall time), same definition as the W0
# reference baseline note - not the per-sample median (that's the Global Constraints gate stat).
AVG_CPU=$(awk -v j="$prev_jiffies" -v c="$CLK_TCK" -v w="$WALL_S" 'BEGIN{ if (w>0) printf "%.1f", 100.0*j/c/w; else print "0.0" }')

python3 - "$MON" "$OUT_DIR/summary.txt" "$capped" "$EXIT_CODE" "$WALL_S" "$AVG_CPU" <<'EOF'
import csv, statistics, sys
mon_path, summary_path, capped, exit_code, wall_s, avg_cpu = sys.argv[1:7]
rows = [r for r in csv.DictReader(open(mon_path)) if r.get('rss_mb')]
lines = [f"wall_s={wall_s} capped={capped} exit_code={exit_code} avg_cpu_pct={avg_cpu}"]
if rows:
    cpu = [float(r['cpu_percent']) for r in rows[1:]]  # skip first partial interval
    rss = [float(r['rss_mb']) for r in rows]
    hwm = [float(r['vmhwm_mb']) for r in rows]
    lines.append(f"samples={len(rows)} sampled_duration_s={rows[-1]['elapsed_s']}")
    if cpu:
        lines.append(f"cpu_median_pct={statistics.median(cpu):.1f} cpu_max_pct={max(cpu):.1f}")
    lines.append(f"rss_first_mb={rss[0]:.0f} rss_final_mb={rss[-1]:.0f} peak_vmhwm_mb={max(hwm):.0f}")
else:
    lines.append("no samples collected")
open(summary_path, 'w').write("\n".join(lines) + "\n")
print("\n".join(lines))
EOF
echo "exit_code=$EXIT_CODE"
tail -5 "$OUT_DIR/run.log"
