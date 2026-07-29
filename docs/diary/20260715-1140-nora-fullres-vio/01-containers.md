# 01 — Containers preparation (VINS-Fusion)

ORB-SLAM3 runs natively on the host (no container). Only **VINS-Fusion** needs Docker (ROS-noetic).

## Image
`dops-kalibr-dev:latest` — the kalibr ROS-noetic dev image, with VINS-Fusion built into a host-mounted
catkin workspace. **The VINS build lives on the host bind-mount, NOT in the image layer** — so any fresh
container from this image + the same mount has a working `vins_node`.

- Host workspace: `~/praca/dev/vins-ws`  (catkin `src/ build/ devel/`; VINS source at `src/VINS-Fusion`)
- Built node: `/root/vins_ws/devel/lib/vins/vins_node` (inside container)
- How VINS-Fusion was originally built (reference): HKUST VINS-Fusion in this image with `apt libceres-dev`
  + OpenCV-3→4 patches. Rebuild after code edits: `catkin_make` in `/root/vins_ws` (see below).

## The two containers used this session
`docker ps` shows both `Up`:

| container      | output mount (`/vins_out`)                    | purpose                        |
|----------------|-----------------------------------------------|--------------------------------|
| `vins-build`   | `~/praca/dev/vins-ws/vins_out`                | legacy IMU-noise runs          |
| `vins-measimu` | `~/praca/dev/vins-ws/vins_out_measimu`        | Allan-measured IMU-noise runs  |

Separate containers/outputs are REQUIRED to run two VINS jobs concurrently: each container is network-
isolated (own roscore), and separate `/vins_out` mounts avoid the shared `vio.csv` race.

## (Re)create a VINS container
```bash
mkdir -p ~/praca/dev/vins-ws/vins_out_measimu   # once, for the measured container
docker run -d --name vins-measimu \
  -v /home/kmro/praca/dev/vins-ws:/root/vins_ws \
  -v /home/kmro/praca/dev/vins-ws/vins_out_measimu:/vins_out \
  -v /home/kmro/praca/dev/orbslam3-eval:/home/kmro/praca/dev/orbslam3-eval \
  -v /media/kmro/datasets:/media/kmro/datasets \
  dops-kalibr-dev:latest sleep infinity
```
The mount `-v <orbslam3-eval>:<same path>` makes host paths valid inside the container (configs/datasets
are referenced by their host absolute path). `vins-build` uses the same mounts but `vins_out` (not `_measimu`).

## Rebuild VINS after editing its C++ (needed for the circular-mask task, see 08)
```bash
docker exec vins-build bash -lc '
  source /opt/ros/noetic/setup.bash
  cd /root/vins_ws && catkin_make -j6 2>&1 | tail -20'
# repeat for vins-measimu, OR (since both mount the same /root/vins_ws) build ONCE — the devel/ is shared.
```
Because both containers bind-mount the SAME `~/praca/dev/vins-ws`, a single `catkin_make` updates the
binary for both. (The source is `src/VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp`.)

## Gotchas
- **`pkill -f vins_node` self-kills the launcher**: `-f` matches the whole command line, and the
  `bash -lc '…'` wrapper string contains "vins_node". Use `pkill -x vins_node` (exact process name).
  Same for roscore/rosmaster/rosout. (Discovered when a run died at exit 143 with no log.)
- `rosbag play` prints a progress bar that floods stdout — always `>/dev/null 2>&1`.
- Full-res bags are large (~8.5 GB for 538's 4197 frames, ~2.6 GB for 542). Watch disk in `~/praca/dev/vins-ws`.
- After a run, drain before copying vio.csv: wait until `wc -l /vins_out/vio.csv` stops growing (VINS may
  lag real-time on full-res), THEN `cp vio.csv vio_<flight>.csv`.
