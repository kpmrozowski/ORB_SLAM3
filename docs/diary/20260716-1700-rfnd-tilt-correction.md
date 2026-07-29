# Rangefinder tilt correction (user spec) — slant range → vertical AGL

**Date:** 2026-07-16 ~17:00.

User: the rangefinder measures the distance to the point along its beam (body-down axis), not
the vertical distance to ground — with roll/pitch the reading exceeds true AGL; correct via
trigonometry (`dist·cos(√(roll²+pitch²))` or equivalent).

## Implementation

- `eval/export_att_csv.py`: scans the ArduPilot BIN (`00000026.BIN`, both flights share it and
  the verified TimeUS↔companion clock bridge in `nora_20260709/lib/nora_bin.py`) for **ATT**
  messages → per-dataset `ref_att.csv` (rebased ns, roll/pitch in radians). 25 977 samples.
- `eval/make_fused_alt.py`: applies **AGL = dist · cos(roll) · cos(pitch)** — the exact
  world-vertical component of the body-down axis; equals the user's `cos(√(roll²+pitch²))` to
  2nd order — interpolated per rangefinder sample, before validity filtering and smoothing.
  Env `--no-tilt` to disable; output versioned as `ref_fused_tilt.csv` (the un-tilted
  `ref_fused.csv` stays for the already-running batch's consistency).

## Measured magnitude — the user was right, it is NOT small

| flight | tilt p50 | tilt p90 | max | slant excess at cruise |
|---|---|---|---|---|
| 542 | 15.4° | 23.0° | 26.9° | ~1.1 m at 24 m AGL (cos p50 0.953) |
| 538 | 17.3° | 26.1° | 30.3° | **~6 m at 130 m AGL** (cos p50 0.955) |

These survey flights cruise at speed with sustained pitch — the correction shifts the whole
rangefinder channel by ~4.5-5% plus attitude-correlated structure (turns show up as AGL bumps
in the uncorrected data, which the texture low-pass was partly eating blindly).

## Validation in flight (batch tilt1)

3× 542 tilt-fused vs 3× 542 plain-baro twins, 3× 538 tilt-fused + loop stack — endpoint gap,
dz, vATE per run in `runs/gap_campaign_summary.csv`.
