# Historical flights, baro-fused campaign (hist_baro2): first <5 m gaps on 5 fps data

**Date:** 2026-07-23 ~00:30. User asks executed: killed the baseline rollouts, added an
early-divergence guard (fork commit 72b277b: `ORB_DIVERGE_VMAX=30`/`ORB_DIVERGE_COUNT=4` —
4 consecutive KFs with |v|>30 m/s stop the run, partial trajectory still saved), removed
the 1.5x config upscale (`*_noup.yaml` variants, native stored resolution), and reran all
52 flights with the proven baro recipe (CLAHE3 + sigma 0.15 + gate 6 + frame-edge 0.5,
deterministic, 5-wide, batch `hist_baro2`).

## Headline

**Three historical 300x225 @5 fps flights meet the <5 m endpoint-gap target:**

| run | cov | vATE | gap |
|---|---|---|---|
| 292_golem17_2025-12-09 | 99.0% | 1.58 m | **2.27 m** |
| 260_golem17_2025-12-01 | 99.3% | 1.56 m | **3.64 m** |
| 173_golem27_2026-02-26 | 98.8% | 2.55 m | **4.17 m** |
| 389_alma231_2025-11-26 | 98.4% | **0.92 m** | 6.00 m |
| 390_alma231_2025-11-27 | 98.1% | 1.48 m | 67.2 m |

15/52 runs reached >=80% coverage (vATE median 7.5 m); 36 were stopped by the divergence
guard; 3 never initialized (335_golem17 dusk flight, 38_epos1, 405_alma231 — the same
dark/low-texture outliers the match check flagged).

## What the guard revealed

The dominant failure is a **velocity blow-up in the first ~1-2 minutes** (guard fires at
frame 65-500 on most casualties, i.e. during/just after IMU init in the climb) — the
known catastrophic-init mode, not gradual drift. Flights that survive init track to the
end. The guard turned each doomed run from ~40 min of wasted compute into ~2-5 min, and
the whole campaign finished in a fraction of the baseline batch's wall-clock.

Baro keeps altitude honest even on runs that later die (vATE before abort: 89_papa3
0.83 m, 106_epos3 0.96 m, 345_golem17 0.99 m); the divergence is horizontal.

## Confound to resolve next

90_golem23 tracked 99.4% of frames with upscale+no-baro (km-scale drift), but with
no-upscale+baro it dies at frame 507. Baro-on and upscale-off changed together for the
low-res sets — an upscale+baro cross on the early-abort flights would separate the two
effects (only if the extra runtime is acceptable).

## Match/feature verification (all 43 low-res flights, OpenCV mirror, upscale profile)

- >=500 matches to >=1 of 4 neighbours: **median 99.5% of frames**; 31/43 datasets >=99%.
- >=10000 features: median 95.4% of frames (upscale profile; native pools ~8.7 k median).
- Failures are content-limited (dusk/winter darkness), not threshold-limited.

## Infrastructure

`eval/det_hist_baro2.txt` (52 runs), `configs/hist/*_noup.yaml`, guard in fork 72b277b,
summary rows batch `hist_baro2` in `runs/det_campaign_summary.csv`, match verdicts in
`eval_out/hist_match/*/frame_verdicts.csv`.
