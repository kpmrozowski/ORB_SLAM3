#!/bin/bash
# ============================================================================================
# NORA golem27 20260709 — FULL-RESOLUTION VIO pipeline (native kalibr calib, post-takeoff start)
# One-shot reproduction of the 2026-07-15 session. Run from ROOT. Idempotent-ish (re-creates
# datasets/configs already committed; skips heavy runs if trajectories already exist).
#
#   ROOT = ~/praca/dev/orbslam3-eval        (all commands are relative to it)
#   Host is shared with HypersearchRunner — check `uptime`, keep <=2 heavy full-res jobs at once.
# ============================================================================================
set -e
ROOT=~/praca/dev/orbslam3-eval
cd "$ROOT"
VINSWS=~/praca/dev/vins-ws

# --------------------------------------------------------------------------------------------
# 0. SOURCES (read-only; NEVER modify /media)
# --------------------------------------------------------------------------------------------
S538="/media/kmro/datasets/dops/vio/20260709/538_golem27_2026-07-09T17-02-39"
S542="/media/kmro/datasets/dops/vio/20260709/542_golem27_2026-07-09T17-26-03.temp"
CACHE="$ROOT/nora_20260709/cache"                     # BIN RISI/GPS/baro/rfnd/mag npz cache
CALIB="/home/kmro/praca/dev/datasets/calibrations/golem27-satic_calib/regression_postchange/camchain.yaml"

# --------------------------------------------------------------------------------------------
# 1. DATASETS — full-res, trimmed to post-takeoff motion, on the SHARED rebased clock.
#    start-ns = user-given "accel jumps +5%" timestamp; epoch-ns = ORIGINAL full-dataset epoch
#    (so nora*_move stays comparable to gps_ref_*.csv / NORA on one clock). Images are 1640x1232
#    SYMLINKS to the originals (nothing under /media copied).
# --------------------------------------------------------------------------------------------
python3 convert_nora_to_euroc.py --session "$S538" --cache "$CACHE" --out dataset/nora538_move \
   --start-ns 1783605978430903808 --end-ns 1783606532881061376 --epoch-ns 1783605974565266432
python3 convert_nora_to_euroc.py --session "$S542" --cache "$CACHE" --out dataset/nora542_move \
   --start-ns 1783607231156786688 --end-ns 1783607365295137536 --epoch-ns 1783607204961902080
# 538 first frame -> rebased 3.87 s (skips ~3.9 s static); 542 -> rebased 26.19 s (skips ~26 s static).
# Validate the trim: accel-std jumps x5.9 (538) / x8.1 (542) at those points (see 02-datasets-convert.md).

# --------------------------------------------------------------------------------------------
# 2. ORB-SLAM3 (host). Configs already written in configs/ (see 03-configs.md). Frame-drop patch
#    ORB_TSJUMP_S=6.0 (flights have 3-4.6 s camera gaps). ORB_STATS_CSV -> per-frame detections +
#    matches_inliers. stdbuf -oL (block-buffered stdout).
# --------------------------------------------------------------------------------------------
source "$ROOT/env.sh"
export ORB_TSJUMP_S=6.0
ORB=$ROOT/ORB_SLAM3 ; VOCAB=$ORB/Vocabulary/ORBvoc.txt
run_orb () {  # mode config dataset tag
  local mode=$1 cfg=$2 ds=$3 tag=$4
  local bin=$ORB/Examples/Monocular/mono_euroc
  [ "$mode" = mono_inertial ] && bin=$ORB/Examples/Monocular-Inertial/mono_inertial_euroc
  local out=$ROOT/runs/$ds/$mode/$tag
  [ -s "$out/f_$tag.txt" ] && { echo "skip $ds/$mode/$tag (done)"; return; }
  mkdir -p "$out"; ( cd "$out"; export ORB_STATS_CSV="$out/frame_stats.csv"
    stdbuf -oL nice -n 10 "$bin" "$VOCAB" "$cfg" "$ROOT/dataset/$ds" \
      "$ROOT/dataset/$ds/cam0_times.txt" "$tag" > "$ROOT/logs/${mode}_${ds}_${tag}.log" 2>&1 )
  echo "$ds/$mode/$tag: $(wc -l < "$out/f_$tag.txt") poses"
}
for DS in nora538_move nora542_move; do
  run_orb mono          configs/nora20260709_mono_native1640.yaml          "$DS" native1640
  run_orb mono_inertial configs/nora20260709_mi_native1640.yaml            "$DS" native1640
  run_orb mono_inertial configs/nora20260709_mi_native1640_measimu.yaml    "$DS" native1640_measimu
done

# --------------------------------------------------------------------------------------------
# 3. VINS-Fusion (container). See 01-containers.md. Legacy noise -> container vins-build/-vins_out;
#    measured noise -> SEPARATE container vins-measimu/-vins_out_measimu (avoids roscore/output races).
#    Build full-res bags from nora*_move, run vins_node + rosbag play, copy vio.csv out, convert to TUM.
# --------------------------------------------------------------------------------------------
run_vins () {  # container out_mount config flight bagsuffix csvsuffix
  local ctr=$1 outmount=$2 cfg=$3 F=$4 bagsfx=$5 csvsfx=$6
  docker exec "$ctr" bash -lc "cd $ROOT/nora_20260709/vins && \
     python make_rosbag.py $ROOT/dataset/nora${F}_move /root/vins_ws/nora${F}_${bagsfx}.bag"
  docker exec "$ctr" bash -lc '
     source /opt/ros/noetic/setup.bash && source /root/vins_ws/devel/setup.bash
     pkill -f vins_node 2>/dev/null; pkill -f roscore 2>/dev/null; sleep 2
     roscore >/tmp/roscore.log 2>&1 & sleep 4
     rosrun vins vins_node '"$cfg"' >/vins_out/vins_'"${F}_${csvsfx}"'.log 2>&1 & sleep 6
     rosbag play -r 1.0 /root/vins_ws/nora'"${F}_${bagsfx}"'.bag >/dev/null 2>&1
     sleep 15; cp /vins_out/vio.csv /vins_out/vio_'"${F}_${csvsfx}"'.csv
     pkill -x vins_node; pkill -x roscore; pkill -x rosmaster; pkill -x rosout; sleep 2'
     # NOTE: use pkill -x <comm> (exact process name), NOT pkill -f vins_node — "-f" matches the
     # whole command line and the bash -lc wrapper CONTAINS "vins_node", so pkill -f SIGTERMs the
     # script itself (exit 143, no log). This bit us once; -x is the fix.
}
# legacy noise (vins-build must be Up; see 01-containers.md to (re)create it)
for F in 538 542; do
  run_vins vins-build "$VINSWS/vins_out" \
     "$ROOT/nora_20260709/vins/nora_vins_1640.yaml" "$F" move move
  python3 eval/vins_to_tum.py "$VINSWS/vins_out/vio_${F}_move.csv" \
     "runs/nora${F}_move/vins_native1640_tum.txt"
done
# measured noise (dedicated container; create it first — see 01-containers.md)
for F in 538 542; do
  run_vins vins-measimu "$VINSWS/vins_out_measimu" \
     "$ROOT/nora_20260709/vins/nora_vins_1640_measimu.yaml" "$F" measimu measimu
  python3 eval/vins_to_tum.py "$VINSWS/vins_out_measimu/vio_${F}_measimu.csv" \
     "runs/nora${F}_move/vins_native1640_measimu_tum.txt"
done

# --------------------------------------------------------------------------------------------
# 4. ANALYSIS + PLOTS (see 06-statistics.md, 07-plots.md). Examples for 538; repeat for 542.
# --------------------------------------------------------------------------------------------
DIARY=docs/diary/20260715-1140-nora-fullres-vio
# combined trajectory figure (horizontal Umeyama-2D + vertical vs baro + matches/frame):
python3 eval/combined_plot.py --gps nora_20260709/gps_ref_538.csv --baro dataset/nora538_move/ref_baro.csv \
   --title "538 full-res moving-start" \
   --traj "ORB-MI=runs/nora538_move/mono_inertial/native1640/f_native1640.txt=tum" \
   --traj "VINS=runs/nora538_move/vins_native1640_tum.txt=tum" \
   --stats "ORB-MI=runs/nora538_move/mono_inertial/native1640/frame_stats.csv" \
   --out $DIARY/plots/combined_538.png
# per-10 s height drift vs baro:
python3 eval/height_drift.py --traj runs/nora538_move/mono_inertial/native1640/f_native1640.txt \
   --baro dataset/nora538_move/ref_baro.csv --window 10 --csv $DIARY/data/height_drift_538_mi.csv
# frame-to-frame ORB match inspection (drawMatches images + curve). stride 1 = every consecutive pair:
python3 eval/match_viz.py --dataset dataset/nora538_move --out $DIARY/plots/matches_538 \
   --low 40 --nfeatures 2000 --stride 1 --title "538 full-res"
# compass fusion on the VINS output (see 05-compass-fusion.md):
python3 eval/mag_fuse_nora.py --traj runs/nora538_move/vins_native1640_tum.txt \
   --gps nora_20260709/gps_ref_538.csv --epoch-ns 1783605974565266432 \
   --out-dir runs/nora538_move/vins_mag
echo "run_all.sh complete."
