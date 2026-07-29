# 05 — Compass / magnetometer fusion (separate handoff)

## Status
The **method is built and validated** on ORB-SLAM3 (prior session); it was NOT yet applied to the new
VINS full-res outputs this session (VINS diverged, so there was nothing good to fuse). The tool is generic
(`eval/mag_fuse_nora.py --traj <any TUM traj>`), so applying it to a VINS trajectory is one command once a
non-diverged VINS run exists.

## What "compass fusion" means here
VINS-Fusion mono-inertial does **NOT** natively fuse a magnetometer. Two ways to add it:
1. **Loosely-coupled, post-hoc (implemented)** — `eval/mag_fuse_nora.py`: take a finished trajectory,
   measure its yaw drift against the magnetometer, and correct it. Same method already run on ORB-SLAM3.
2. **Tightly-coupled, in-estimator (NOT done)** — VINS ships a `global_fusion` node (built:
   `~/praca/dev/vins-ws/devel/lib/global_fusion/global_fusion_node`, source `globalOpt.cpp`) that fuses
   VIO odometry with a global sensor via a pose graph (designed for GPS). A magnetometer heading factor
   could be added there (C++ + rebuild). Heavier; only worth it if the loosely-coupled result is promising.

## How the loosely-coupled fusion works (`eval/mag_fuse_nora.py`)
Pipeline (functions in the file):
1. `load_tum_full(traj)` → times, positions, quaternions (the VIO body orientation per pose).
2. Load the BIN-calibrated magnetometer in the **body frame** from `nora_20260709/cache/mag_calib_body.npz`
   (hard-iron [142,70,22] mG + soft-iron DIA/ODI + ORIENT=101 custom rotation, from the BIN COMPASS_* params
   via the compass-calibration project's reader; timestamps via `nora_bin.timeus_to_companion_ns`).
3. `align_mag_to_body(rotations, mag_body)` — iterative Kabsch/Wahba fit of a CONSTANT rotation that makes
   `R_wb · R_align · mag_body` constant in the world frame. Returns the alignment + `world_std`.
4. `drift_from_mag(mag_world, up_axis)` — the azimuth wobble of the world-frame field = the VIO yaw drift.
5. `reintegrate(positions, up_axis, drift)` — re-rotate the position increments by −drift to remove yaw drift.

Key **finding** (corrects an earlier "heading drift" guess): after the constant alignment, `mag_world` is
constant to per-axis std **0.03** → **the VIO attitude is genuinely good**; measured heading drift is only a
bounded ±20°/23°-range wiggle over 9 min. So mag fusion moved the ORB-SLAM3 538 GPS RMSE only 308→301 m
(**2 %**). The real 538 residual is monocular horizontal SCALE drift, which the magnetometer cannot fix.
The mag's genuine value: (a) it CONFIRMS attitude is accurate, (b) absolute-north reference for GPS-denied use.

## Apply it to a (non-diverged) VINS trajectory
```bash
python3 eval/vins_to_tum.py ~/praca/dev/vins-ws/vins_out/vio_538_move.csv runs/nora538_move/vins_native1640_tum.txt
python3 eval/mag_fuse_nora.py \
   --traj runs/nora538_move/vins_native1640_tum.txt \
   --gps nora_20260709/gps_ref_538.csv \
   --epoch-ns 1783605974565266432 \        # 538 rebase epoch (542: 1783607204961902080)
   --mag-cache nora_20260709/cache/mag_calib_body.npz \
   --out-dir runs/nora538_move/vins_mag
```
Produces the corrected trajectory + a report of `world_std` (attitude quality) and the RMSE change.
**Prerequisite: a VINS run that does not diverge** — so this waits on fixing VINS init (see RESUME).
