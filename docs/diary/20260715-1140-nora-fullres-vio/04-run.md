# 04 — How to run every estimator

`run_all.sh` in this directory does the whole pipeline. Below is the per-estimator detail.

## Host / shared-machine hygiene
- Shared with the user's `HypersearchRunner --max-workers 30` (load spikes to 400+). Check `uptime`;
  keep **≤2** full-res jobs concurrent; prefix heavy runs with `nice -n 10`.
- ORB-SLAM3 stdout is block-buffered → wrap in `stdbuf -oL` or "hangs" look like starvation.

## ORB-SLAM3 (host)
```bash
source ~/praca/dev/orbslam3-eval/env.sh          # sets LD_LIBRARY_PATH for the prebuilt deps
export ORB_TSJUMP_S=6.0                            # frame-drop patch: don't reset map on <6 s gaps
ORB=~/praca/dev/orbslam3-eval/ORB_SLAM3
VOCAB=$ORB/Vocabulary/ORBvoc.txt
OUT=~/praca/dev/orbslam3-eval/runs/<dataset>/<mode>/<tag>; mkdir -p "$OUT"; cd "$OUT"
export ORB_STATS_CSV="$OUT/frame_stats.csv"        # per-frame: timestamp,state,detections,matches_inliers,grab_ms
BIN=$ORB/Examples/Monocular/mono_euroc             # or Monocular-Inertial/mono_inertial_euroc
stdbuf -oL "$BIN" "$VOCAB" <config.yaml> ~/praca/dev/orbslam3-eval/dataset/<dataset> \
   ~/praca/dev/orbslam3-eval/dataset/<dataset>/cam0_times.txt <tag>  > $ROOT/logs/<...>.log 2>&1
# Outputs (written AT PROCESS EXIT): f_<tag>.txt (per-frame TUM traj), kf_<tag>.txt (keyframes).
```
Env vars the binaries read: `ORB_TSJUMP_S` (frame-drop threshold s), `ORB_STATS_CSV` (stats path),
`ORB_VIEWER=1` (Pangolin GUI, needs DISPLAY). Wrapper scripts exist: `run_dataset.sh`, `run_stats.sh`.

**⚠ Empty-map save crash:** if the final active map is empty (pervasive resets), `SaveTrajectoryEuRoC`
throws `std::system_error` and NO `f_*.txt` is written — the run "completed" but produced nothing. This
happened for every native-1640 MI run this session. `frame_stats.csv` still survives (use it for match stats).

## VINS-Fusion (container) — see 01-containers.md for container setup
```bash
# 1. build a full-res bag (inside container; make_rosbag.py skips negative-stamp IMU pre-roll)
docker exec vins-build bash -lc 'cd /home/kmro/praca/dev/orbslam3-eval/nora_20260709/vins && \
   python make_rosbag.py /home/kmro/praca/dev/orbslam3-eval/dataset/nora538_move /root/vins_ws/nora538_move.bag'
# 2. run: roscore + vins_node + rosbag play, then drain + copy vio.csv
docker exec vins-build bash -lc '
   source /opt/ros/noetic/setup.bash && source /root/vins_ws/devel/setup.bash
   pkill -x vins_node; pkill -x roscore; sleep 2
   roscore >/tmp/roscore.log 2>&1 & sleep 4
   rosrun vins vins_node /home/kmro/praca/dev/orbslam3-eval/nora_20260709/vins/nora_vins_1640.yaml \
      >/vins_out/vins_538_move.log 2>&1 & sleep 6
   rosbag play -r 1.0 /root/vins_ws/nora538_move.bag >/dev/null 2>&1
   sleep 15; cp /vins_out/vio.csv /vins_out/vio_538_move.csv
   pkill -x vins_node; pkill -x roscore; sleep 2'
# 3. vio.csv -> TUM for the eval tooling
python3 eval/vins_to_tum.py ~/praca/dev/vins-ws/vins_out/vio_538_move.csv \
   runs/nora538_move/vins_native1640_tum.txt
```
- **`pkill -x`, never `pkill -f`** (see 01 — `-f` self-kills the wrapper).
- Measured-noise runs: same but container `vins-measimu`, config `nora_vins_1640_measimu.yaml`, output
  lands in `~/praca/dev/vins-ws/vins_out_measimu/`.
- vio.csv columns: `t_ns, px, py, pz, qw, qx, qy, qz, vx, vy, vz` (quaternion **w-first**; TUM is w-last).

## Delegating runs (what worked / didn't this session)
Runs were delegated to Opus subagents. **Caveat**: subagents tended to launch jobs with `nohup &`
(detached) then end their turn, so they didn't reliably collect results or start the 2nd flight. If
delegating again, instruct them to run FOREGROUND (blocking) and not background/detach, or just drive the
runs from the main context with a background waiter on the output files (what ended up working).
