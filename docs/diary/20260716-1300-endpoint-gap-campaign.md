# Endpoint-gap campaign — metric, baselines, fused baro+rfnd, PR-sensitivity knobs

**Date:** 2026-07-16 ~13:00. New prime metric (user): **takeoff→landing distance** —
target **<5 m (538), <2 m (542)**. Both flights land on the takeoff spot, so
||p_end − p_start|| is a frame-invariant closure metric needing no GT alignment.

## Metric + baseline census (`eval/endpoint_gap.py`, 1 s endpoint medians)

| trajectory | gap [m] | note |
|---|---|---|
| NORA EKF 538 | **84.7** | spoof-GPS flights — the EKF has no absolute anchor either |
| NORA EKF 542 | **46.3** | dito — **closure is where SLAM can beat NORA** |
| best 542 run (n7_s0075_f05_r15) | **3.42** (dz 2.13) | already 13× better than NORA; target 2.0 |
| best 538 run (t538_s0075_f05_r6) | **61.8** (dz 12.5) | better than NORA; target 5.0 — needs loops |
| 542 no-baro (clahe_nf5000) | 13.3 | baro fusion already helps closure |

Decomposition: 542's 3.42 = ~2.7 horizontal + ~2.1 vertical; 538's ~62-73 = mostly horizontal
(monocular yaw/position drift over 6.5 km of survey), vertical 12-38.

## Vertical gap root cause — the barometer's landing dip

The baro is rest-to-rest consistent (−2.05 → −2.22 m over the whole 542 flight) but during the
final two seconds it dips to **−6.9 m** (ground-effect/prop-wash transient, t≈154.6 s; 538
analogous). The σ-lagged z-fusion partially bakes that dip into the final SLAM poses → the
~2-3 m vertical endpoint gap. NORA's EKF suffers it too (its own landing window −3.6 m).

## Fused baro+rangefinder reference (`eval/make_fused_alt.py` → `dataset/*/ref_fused.csv`)

Per user spec: **w(rfnd)=1 below 10 m AGL, w(baro)=1 above 80 m**, linear in between.
Design points learned the hard way:
- rfnd validity: [0.3, 200] m + 1 s-median spike filter + 2 s centered mean (zero-phase);
- **datum from pre-motor rest**: baro head (first 3 s) minus rfnd mount reading (~0.12-0.16 m).
  The rfnd log starts ~1 s pre-takeoff with motors already running — prop wash corrupts an
  "on-ground shared-window" datum by ~2 m;
- **correction-hold**: the rfnd correction to baro is interpolated across uncovered stretches and
  edge-held from 3 s medians — v1 snapped to baro at touchdown (where rfnd goes below its 0.3 m
  floor), v2 held the single worst transient sample; both wrong;
- result: the −6.9 m landing dip is fully removed (fused descends smoothly through touchdown);
  rest-end −1.7 m closer to truth on both flights. Drop-in via `ORB_BARO_CSV=ref_fused.csv`.

Caveat recorded: anchoring the landing to rfnd-truth can *worsen* vATE-vs-raw-baro (the baro GT
itself is wrong in the landing window) while improving the real gap — gap wins per user priority.

## Place-recognition sensitivity (`ORB_PR_NCAND/COINC/MATCH_SCALE`, commit a18ca5a)

Knobs work (defaults stock; scale floors at 1) — but **zero detections in 6/6 runs, even
aggressive** (NCAND 10, COINC 1, SCALE 0.4, gap unchanged 65-104 on healthy draws). The cascade
dies before any tunable threshold. `ORB_PR_DEBUG` instrumentation added (commit 87e18ea) at the
four gates (candidates → BoW matches → Sim3 RANSAC → projection); autopsy runs in flight.

## In flight (batch pr2)

2× PRDBG autopsy (538+542) + 3×542-fused + 3×538-fused (unified recipe, gap metric now in every
queue-runner eval line: `eval/night_queue4.sh`, summary `runs/gap_campaign_summary.csv`).
