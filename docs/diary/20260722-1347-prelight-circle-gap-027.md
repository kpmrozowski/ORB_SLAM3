# circle_move_prelight_junk: endpoint gap 0.27 m — circle target SMASHED

**Date:** 2026-07-22 13:47. User ask: test the best 538/542 params on `circle_move_prelight_junk`
(circle_move / circle_move2 removed — they were "just a moment before landing"; the real circle
flight lives in the earlier 148-s segment I had mislabeled "prelight junk").

## Setup

- Dataset: 2935 frames / 148.4 s / IMU 374.7 Hz, 800x600, config `circle_mi_800native_nf5000.yaml`.
- The circle source (data-recorder format: cam0 + imu0.csv only) has **no baro and no mag** —
  the 538/542 recipes therefore collapse to their common core:
  `ORB_DETERMINISTIC=1 ORB_NO_PACE=1 OMP_NUM_THREADS=1 ORB_CLAHE=3.0` (nf5000, stock init gates).
- Batch `prelight1` via `eval/det_queue.sh` (queue `eval/det_prelight.txt`), 5 runs parallel.

## Results (`runs/det_campaign_summary.csv`, batch prelight1)

| arm | gap [m] | dz [m] | cov | md5 |
|---|---|---|---|---|
| pj_core_r1/r2/r3 (CLAHE3, stock gates) | **0.27** | 0.21 | 99.4% | **bit-identical ×3** (52a1c2c9) |
| pj_gates_a (shortened init gates) | 1.07 | 0.71 | 99.4% | c086707c |
| pj_noclahe | 200.73 | 111.60 | 99.7% | 72a79192 |

Sanity of the winner: single surviving map spanning the whole flight (2 resets are pre-init
warm-up), VIBA1+VIBA2 complete, ~10 m altitude, 169 m path length, max pose gap 0.22 s.

## Take-aways

1. **Endpoint gap 0.27 m ≪ 5 m target** — the first dataset where the closure target is met,
   and it needed no baro/mag at all.
2. **Determinism holds on a fresh dataset**: 3/3 runs bit-identical (no MALLOC_PERTURB) —
   the IdLess+EIGEN_DONT_VECTORIZE+sequential stack generalizes; the open uninit-read jitter
   did not trigger here.
3. **CLAHE3 is load-bearing**: without it the same flight produces a 200-m gap (scale/drift
   blow-up), despite slightly higher coverage. Consistent with the 07-15 match-quality finding.
4. Shortened init gates (validated on the 14-s clip) are **counterproductive on a full-length
   flight** (1.07 vs 0.27) — keep stock gates whenever the flight is long enough.
5. The earlier "no circle flight exists" verdict applied to the *final-11-s* conversion only;
   the user's pointer to the prelight segment resolved the mystery — takeoff, circle and
   landing are all inside it.
