# 03 — Configs (how they were created)

## Native full-res calibration (the reason for this session)
`/home/kmro/praca/dev/datasets/calibrations/golem27-satic_calib/regression_postchange/camchain.yaml`
(kalibr, camera-only — NO imu block):
```
camera_model: pinhole, distortion_model: equidistant   # == Kannala-Brandt / KB8 fisheye
resolution: [1640, 1232]
intrinsics:        [570.3874069240171, 571.3253391403026, 788.9216052932007, 637.1006809202664]  # fx fy cx cy
distortion_coeffs: [0.03742367684054015, -0.00461792825796154, 0.005166909928927371, -0.002256513353191578]  # k1..k4
```
The old 1640 configs used **scaled** 800 intrinsics (fx 575, cx 798.9) + the **800-fit distortion**
(k1 0.048…) → that combo tracked only 11 %. The native distortion (k1 0.037…) is the fix. Map:
kalibr equidistant `distortion_coeffs[0..3]` → ORB `Camera1.k1..k4` and VINS camodocal `k2..k5` directly;
`intrinsics` → ORB `fx/fy/cx/cy`, VINS `mu/mv/u0/v0`.

## Camera-IMU extrinsic + IMU frequency (resolution-independent → reused)
`T_b_c1` (= inv(camchain T_cam_imu)) is a physical transform, independent of image resolution, so it is
copied verbatim from `configs/nora20260709_mi_800.yaml`:
```
data: [ 0.007479433982916485, 0.9999705722590355, 0.001706658498651807, 0.01307281636339634,
       -0.99964387264817,     0.007520689029557638,-0.02560404486577578,-0.1464888897324127,
       -0.02561612664442386, -0.001514546947611002, 0.9996707058843323,  0.2019611555492663,
        0.0,0.0,0.0,1.0]
IMU.Frequency: 400
```

## IMU noise — two variants
| param (ORB / VINS)        | LEGACY (baseline)      | Allan-variance MEASURED (this session)      |
|---------------------------|------------------------|---------------------------------------------|
| NoiseGyro / gyr_n         | 5.0e-4                 | 3.65e-5   (avg of per-axis ND rad/s/√Hz)    |
| NoiseAcc  / acc_n         | 1.3e-2                 | 4.7175e-3 (avg accel ND 4.7175 mm/s²/√Hz)   |
| GyroWalk  / gyr_w         | 4.0e-6                 | 1.38e-6   (avg gyro RW rad/s²/√Hz)          |
| AccWalk   / acc_w         | 9.0e-4                 | 4.0e-5    (avg accel RW 3.998e-2 mm/s³/√Hz) |
Measured = the user's Allan summary (2 h, 200 Hz), averaged over axes. The Z accel axis is ~30× noisier
than X/Y; averaging is the scalar the estimators want. **Result: measured is strictly worse here** (see 06)
— tighter noise over-trusts an imperfect IMU/extrinsic. Legacy is the keeper (matches the standing memory
`project_imu_per_axis_noise`: heavy IMU weight degrades results when the extrinsic is imperfect).

## Config files written (in the repo, committed)
ORB-SLAM3 (`configs/`), format = KannalaBrandt8, `Camera.width/height 1640/1232`, `ORBextractor.nFeatures 2500`:
- `nora20260709_mono_native1640.yaml`      — mono, native calib
- `nora20260709_mi_native1640.yaml`        — mono-inertial, native calib + T_b_c1 + LEGACY noise
- `nora20260709_mi_native1640_measimu.yaml`— mono-inertial, native calib + T_b_c1 + MEASURED noise

VINS-Fusion (`nora_20260709/vins/`), camodocal KANNALA_BRANDT + estimator yaml (fixed extrinsic
`body_T_cam0`, `td 0`, `g_norm 9.7525`, `max_cnt 300`, `min_dist 30` — scaled up from 800's 200/15 for
the 4.5× pixels):
- `nora_cam0_1640.yaml`         — native intrinsics/distortion, 1640×1232
- `nora_vins_1640.yaml`         — mono-inertial, LEGACY noise (acc_n 0.02 gyr_n 0.002 acc_w 0.001 gyr_w 0.0001)
- `nora_vins_1640_measimu.yaml` — mono-inertial, MEASURED noise (acc_n 0.0047175 gyr_n 3.65e-5 acc_w 4e-5 gyr_w 1.38e-6)

## To create an 800-res equivalent (baseline that actually works)
The known-good baseline is `configs/nora20260709_mi_800.yaml` on `dataset/nora538_half` (see
`nora_20260709/NORA_RESULTS.md`). If full-res keeps regressing, fall back to 800 for real results.
