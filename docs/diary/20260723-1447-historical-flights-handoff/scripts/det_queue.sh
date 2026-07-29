#!/bin/bash
# Deterministic-mode batch runner (ORB_DETERMINISTIC): single-threaded SLAM processes,
# safe to run several in parallel (no in-process concurrency -> no g2o race).
# Usage: det_queue.sh <queue.txt> <batch> [parallel]
# Queue line: tag|dataset|config|ENV;ENV  ("-" = none). Gap always evaluated; vATE/drift
# evaluated when the dataset has ref_baro.csv.
ROOT=/home/kmro/praca/dev/orbslam3-eval
source "$ROOT/env.sh"
set -u
QUEUE="$1"; BATCH="$2"; PAR="${3:-5}"
BIN=$ROOT/ORB_SLAM3/Examples/Monocular-Inertial/mono_inertial_euroc
VOCAB=$ROOT/ORB_SLAM3/Vocabulary/ORBvoc.txt
SUMMARY=$ROOT/runs/det_campaign_summary.csv
mkdir -p "$ROOT/logs"
[ -f "$SUMMARY" ] || echo "batch,tag,exit,poses,cov_pct,vate_m,drift_rms_m,gap_m,gap_dz_m,md5" > "$SUMMARY"

run_one() {
    local tag="$1" dataset="$2" config="$3" envspec="$4"
    local ds_dir="$ROOT/dataset/$dataset"
    local out_dir="$ROOT/runs/$dataset/mono_inertial/$tag"
    mkdir -p "$out_dir"
    (
        cd "$out_dir"
        rm -f frame_stats.csv
        export ORB_DETERMINISTIC=1 ORB_NO_PACE=1 OMP_NUM_THREADS=1 ORB_TSJUMP_S=6.0
        export ORB_STATS_CSV="$out_dir/frame_stats.csv"
        unset ORB_CLAHE ORB_CLAHE_TILE ORB_BARO_CSV ORB_BARO_SIGMA ORB_BARO_GATE \
              ORB_BARO_SCALEREF_S ORB_BARO_FRAME_SIGMA ORB_MAG_CSV ORB_MAG_SIGMA_DEG \
              ORB_MASK_FILE ORB_LOOP_RP_TOL ORB_LOOP_YAW_TOL ORB_PR_NCAND ORB_PR_COINC \
              ORB_PR_MATCH_SCALE ORB_PR_SIM3_MININL ORB_PR_FREE_SCALE ORB_PR_DEBUG ORB_DET_DEBUG
        if [ "$envspec" != "-" ]; then
            local IFS=';'
            for assignment in $envspec; do export "$assignment"; done
        fi
        setarch "$(uname -m)" -R nice -n 10 "$BIN" "$VOCAB" "$ROOT/configs/$config" "$ds_dir" \
            "$ds_dir/cam0_times.txt" "$tag" > "$ROOT/logs/det_${tag}.log" 2>&1
        echo "$?" > "$out_dir/exit_code"
    )
    local exit_code; exit_code=$(cat "$out_dir/exit_code" 2>/dev/null || echo 99)
    local traj="$out_dir/f_$tag.txt"
    local poses=0 cov="0" vate="nan" drift="nan" gap="nan" gap_dz="nan" md5="-"
    local nframes; nframes=$(grep -c . "$ds_dir/cam0_times.txt")
    if [ -s "$traj" ]; then
        poses=$(grep -c . "$traj"); cov=$(python3 -c "print(f'{100.0*$poses/$nframes:.1f}')")
        md5=$(md5sum "$traj" | cut -d' ' -f1)
        local gl; gl=$(python3 "$ROOT/eval/endpoint_gap.py" --traj "$traj" 2>/dev/null | tail -1)
        gap=$(echo "$gl" | awk '{print $2}'); gap_dz=$(echo "$gl" | awk '{print $5}')
        if [ -f "$ds_dir/ref_baro.csv" ]; then
            vate=$(python3 "$ROOT/eval/baro_ate.py" --baro "$ds_dir/ref_baro.csv" \
                --traj "T=$traj=tum" --title x --out "$out_dir/vate.png" 2>/dev/null \
                | awk '$1=="T" {print $2}')
            drift=$(python3 "$ROOT/eval/height_drift.py" --traj "$traj" --kind tum \
                --baro "$ds_dir/ref_baro.csv" 2>/dev/null | awk '/per-window drift/ {gsub(",",""); print $4}')
        fi
        vate=${vate:-nan}; drift=${drift:-nan}; gap=${gap:-nan}; gap_dz=${gap_dz:-nan}
    fi
    echo "$BATCH,$tag,$exit_code,$poses,$cov,$vate,$drift,$gap,$gap_dz,$md5" >> "$SUMMARY"
    echo "EVAL $tag exit=$exit_code poses=$poses cov=${cov}% vATE=$vate gap=$gap dz=$gap_dz md5=${md5:0:8}"
}

active=0
while IFS='|' read -r tag dataset config envspec; do
    case "$tag" in ''|'#'*) continue;; esac
    if [ "$active" -ge "$PAR" ]; then wait -n; active=$((active-1)); fi
    run_one "$tag" "$dataset" "$config" "$envspec" &
    active=$((active+1))
done < "$QUEUE"
wait
echo "BATCH $BATCH DONE"
