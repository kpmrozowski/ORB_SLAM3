# 08 — Pending tasks & copy-paste-ready plans

Priority order in RESUME.md. The two NEW user asks (circular mask, IMU-align viz) are first.

═══════════════════════════════════════════════════════════════════════════════════════════════
## TASK 1 — Circular detection mask (filter the sky). "Detect only inside a circle at the principal
## point, diameter = 70 % of image height." Pass a mask to detection (don't post-filter). Rerun all.
═══════════════════════════════════════════════════════════════════════════════════════════════
Geometry (native 1640×1232): center = principal point (cx=788.9216, cy=637.1007);
radius = 0.5 · 0.70 · 1232 = **431.2 px**. Env-configurable so it also works at 800 (cx 389.7, cy 314.3).

### 1a. ORB-SLAM3 — `ORB_SLAM3/src/ORBextractor.cc`, in `operator()` right AFTER line 1100
`ComputeKeyPointsOctTree(allKeypoints);`. Reject keypoints outside the circle before descriptors are
computed (= "detect only there"; the sky keypoints never become map points). Self-contained, env-driven
(same pattern as ORB_TSJUMP_S). `mvScaleFactor[level]` maps level coords → level-0 pixels.
```cpp
// --- circular detection mask (sky filter): keep keypoints inside a circle at the principal point,
//     diameter = ORB_MASK_FRAC * image height. Applied during extraction (before descriptors). ---
static const float maskFrac = getenv("ORB_MASK_FRAC") ? atof(getenv("ORB_MASK_FRAC")) : 0.0f;
if (maskFrac > 0.0f) {
    const float mcx = getenv("ORB_MASK_CX") ? atof(getenv("ORB_MASK_CX")) : image.cols * 0.5f;
    const float mcy = getenv("ORB_MASK_CY") ? atof(getenv("ORB_MASK_CY")) : image.rows * 0.5f;
    const float radius2 = powf(0.5f * maskFrac * image.rows, 2.0f);
    for (int level = 0; level < nlevels; ++level) {
        const float s = mvScaleFactor[level];
        std::vector<cv::KeyPoint> kept; kept.reserve(allKeypoints[level].size());
        for (const cv::KeyPoint& kp : allKeypoints[level]) {
            const float dx = kp.pt.x * s - mcx, dy = kp.pt.y * s - mcy;
            if (dx*dx + dy*dy <= radius2) kept.push_back(kp);
        }
        allKeypoints[level].swap(kept);
    }
}
```
Rebuild: `cd ~/praca/dev/orbslam3-eval/ORB_SLAM3 && source ../env.sh && cmake --build build -j6`
(or the project's build dir — check `ls ORB_SLAM3/build*/`; the binaries were built there).
Run with the mask: `export ORB_MASK_FRAC=0.70 ORB_MASK_CX=788.9216 ORB_MASK_CY=637.1007` before the binary.
(Optional stronger version: also skip whole FAST grid cells outside the circle in
`ComputeKeyPointsOctTree` for speed — not required for correctness.)

### 1b. VINS-Fusion — `.../VINS-Fusion/vins_estimator/src/featureTracker/feature_tracker.cpp`, `setMask()`
After `mask = cv::Mat(row, col, CV_8UC1, Scalar(255));` (line ~57), AND the circle in:
```cpp
{   // sky mask: keep only a circle at the principal point, diameter 0.70 * image height
    static cv::Mat sky;
    if (sky.empty()) {
        sky = cv::Mat::zeros(row, col, CV_8UC1);
        // principal point: use the camera model; fallback to image centre.
        double pp[2] = {col * 0.5, row * 0.5};                 // TODO read u0/v0 from m_camera params
        cv::circle(sky, cv::Point(cvRound(pp[0]), cvRound(pp[1])), cvRound(0.35 * row), 255, -1);
    }
    mask &= sky;
}
```
Also drop already-tracked points outside the circle (KLT ignores the mask): in the tracking step where
`reduceVector` is applied, add a status test `sky.at<uchar>(cur_pts[i]) > 0`. Rebuild:
`docker exec vins-build bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/vins_ws && catkin_make -j6'`
(both containers share `/root/vins_ws`, so one build covers both).

### 1c. Rerun all
`bash docs/diary/20260715-1140-nora-fullres-vio/run_all.sh` with `ORB_MASK_FRAC` exported (add it near the
top of the ORB section). Use NEW tags (e.g. `native1640_mask`) so results don't overwrite the no-mask runs.
Then regenerate stats/plots (06/07). Expected benefit: fewer sky/cloud features → fewer spurious matches,
possibly steadier init. **Verify** with `match_viz` + `frame_stats` before/after.

═══════════════════════════════════════════════════════════════════════════════════════════════
## TASK 2 — IMU-aligned frame debug viz + noise-dependent match search region
## "Align next frame to previous using integrated IMU; show a search region for matches that depends
## on the noises (if such a concept exists)."  → YES, this is standard guided-matching / gyro-prediction.
═══════════════════════════════════════════════════════════════════════════════════════════════
The concept EXISTS: tightly-coupled VIO predicts each feature's next location from the IMU-integrated
relative rotation, and searches for its match within a window sized by the prediction covariance (which
grows with IMU noise). Deliverable: `eval/imu_align_viz.py` producing, for chosen consecutive pairs:
1. frame k-1, frame k, and **frame k IMU-warped into k-1** (should overlay if ΔR is right) — a blended image.
2. per-feature predicted location (yellow ✕) + **search circle** whose radius = propagated IMU-noise σ,
   drawn for BOTH noise sets (legacy vs measured) to visualize how noise sets the search size.
3. matches actually found within the predicted circle (green lines).

### Math
- Relative rotation from gyro: between image stamps t0,t1, `ΔR = Π exp((ω_i - b_g)·dt_i)` over IMU rows in
  (t0,t1] from `dataset/<ds>/mav0/imu0/data.csv` (cols gyro x,y,z). Body→cam via `T_b_c1` (use its rotation
  `R_cb`): `ΔR_cam = R_cb · ΔR_body · R_cb^T`.
- Predicted pixel: unproject p_{k-1} to a unit bearing with the **KB fisheye** model (the native calib —
  NOT a pinhole homography), rotate by ΔR_cam, reproject: `p_k_pred = project(ΔR_cam · unproject(p_{k-1}))`.
  Use the camchain intrinsics f=[570.39,571.33] c=[788.92,637.10] k=[0.0374,-0.00462,0.00517,-0.00226].
- Frame warp: build cv2 remap maps by, for every pixel of the k-1 canvas, unproject→rotate by ΔR_cam^T→
  project into k, then `cv2.remap(frame_k, mapx, mapy)`.
- Search radius (noise → pixels): rotation-angle uncertainty over Δt is
  `σθ = sqrt(gyr_n²·Δt + (gyr_w)²·Δt·T + σ_b0²·Δt²)` (white + bias RW + init-bias term; T = time since
  bias set). Pixel radius `r ≈ kσ · f · σθ` (f≈570; small-angle → dpix/dθ≈f). So legacy gyr_n 5e-4 gives a
  larger circle than measured 3.65e-5 — that contrast is the point. Add a translation/parallax term for
  near scenes; for this high-altitude data pure-rotation dominates.

### Skeleton (finish on resume)
```python
# eval/imu_align_viz.py  — inputs: --dataset, --t (pick a frame time), --pair-dt, --out, noise flags
#   1. load cam0_times + imu0/data.csv; find frames bracketing --t; integrate gyro -> dR_body -> dR_cam
#   2. KB unproject/project (reuse camchain intrinsics); build remap -> warp frame_k into k-1; blend & save
#   3. ORB-detect in k-1; for each kp: predict p_k via dR_cam; radius from sigma(gyr_n,gyr_w,dt) for each
#      noise set; draw predicted-x + circle(s); search ORB match within circle; draw found matches
#   4. save panel PNG(s): [k-1 | k | warped-k-overlay] and [k-1 with predicted pts+search circles]
```
Reuse: `nora_20260709/lib/nora_bin.py` (IMU load helpers), `eval/match_viz.py` (ORB detect/match), the
KB math (write a small `kb_project/kb_unproject` or use `cv2.fisheye`). Legacy vs measured noise values
from 03-configs.md. This directly visualizes "search region depends on noise."

═══════════════════════════════════════════════════════════════════════════════════════════════
## Other pending (were in flight before the mask/viz asks)
═══════════════════════════════════════════════════════════════════════════════════════════════
- **Fix 542 dataset** (56 zero-byte frames) — patch converter, rebuild `nora542_move` (see 02). BLOCKS all 542.
- **VINS divergence** — still diverges (538 889 km, 542 42 km). See RESUME "ideas". Not solved by full-res/trim.
- **Combined plots / drawMatches / VINS+compass / legacy-vs-measured writeup** — tooling ready (06/07/05);
  blocked on runs that actually save a trajectory (native-1640 MI saved none; either fix init or use 800-res).
