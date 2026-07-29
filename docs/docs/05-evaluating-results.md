# 05 — Evaluating Results

Trajectories are scored against onboard sensors (barometer, magnetometer) and, where available, the **authentic** GPS receiver. (On these vehicles GPS **instance 0** is spoofed and GPS **instance 1** is authentic — "GPS[0]/GPS[1]" mean receiver *instances*, not sample indices.)

## Metrics at a glance

| Metric | Measures | Script |
|---|---|---|
| **Coverage %** | tracked poses / total frames | `score_trial.py`, `det_queue.sh` |
| **Endpoint gap** | takeoff→landing closure distance (reset-free runs only) | `eval/endpoint_gap.py` |
| **Vertical ATE** | height error vs barometer (1-DoF) | `eval/baro_ate.py` |
| **Height drift** | per-10s window vertical drift vs baro | `eval/height_drift.py` |
| **ATE** | full-3D error vs a reference (Horn/Umeyama) | `evaluation/evaluate_ate_scale.py` |
| **GPS segment errors** | per-60s residual rotation / offset / scale vs GPS | `eval/hist_gps_segment_errors.py` |
| **J-score** | one composite number per campaign (lower = better) | `eval/ic_tune/score_trial.py` |

Precise formulas and the exact Umeyama point selection are in [`../DEVELOPMENT_REPORT.md` §3](../DEVELOPMENT_REPORT.md#3-evaluation-methodology--metrics).

## The three you'll use most

**Coverage** — `100 · poses / frames`. `< 25%` = diverged, `< 80%` = penalized. Pre-takeoff static frames are unreachable, so ~77% can mean 100% of the airborne span.

**Endpoint gap** — `‖median(last 1s) − median(first 1s)‖`. Frame-invariant, no alignment needed, but **only valid for single-map (reset-free) runs**. Targets: `< 5 m` (long flights), `< 2 m` (short low flights).

**Vertical ATE** — recovers the SLAM "up" axis by least-squares, then RMSE of `(SLAM height − baro)` with **no scale fit** (true metric vertical error).

## The Umeyama alignment (which points)

The horizontal/GPS score fits its transform on the **full per-frame trajectory** `f_*.txt` (all poses, *not* keyframes) against **unspoofed GPS-instance-0** samples, time-interpolated onto trajectory stamps (within 1.5 s). Rotation (yaw) comes from the **magnetometer**, tilt from a full-flight free Umeyama, scale from fixed-rotation least-squares; then a fresh Umeyama per 60-s segment gives the scored residuals. Because centroids are removed, the ENU origin is irrelevant. (See report §3.5.)

## Running the scorers

```bash
# One trial across the flight set → J-score + ledger row
python3 eval/ic_tune/score_trial.py --trial my_run --objective v2

# Standalone metrics on a single trajectory
python3 eval/endpoint_gap.py  f_my_run.txt
python3 eval/baro_ate.py      f_my_run.txt  <dataset>/ref_baro.csv
python3 eval/hist_gps_segment_errors.py  <flight_key>
```

## Reproducibility

Score A/B experiments in **deterministic mode** (`ORB_DETERMINISTIC=1`): on the same machine + commit, `md5sum f_*.txt` is bit-identical run-to-run, so any change in the trajectory is a real effect, not thread noise. (Cross-architecture md5s will differ — compare metrics, not hashes, across x86 vs ARM.) See [doc 06 §Determinism](06-vanilla-algo-all-good-mods.md).
