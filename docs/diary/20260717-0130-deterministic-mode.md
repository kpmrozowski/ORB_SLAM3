# Deterministic mode — the four nondeterminism sources of ORB-SLAM3, found one by one

**Date:** 2026-07-17 ~01:30 (night session). User ask: quantify endpoint-gap determinism
(10× runs on 538/542/circle) and make the algorithm fully deterministic without losing quality.

## Measured baseline nondeterminism (threaded stock architecture)

Same recipe, same machine: 542 endpoint gap ranged **3.4 → 53 m** across draws; 538 62 → 133 m
plus a ~10-35% catastrophic-init lottery. Order-of-magnitude spread — the draw lottery has been
the dominant "error bar" of the whole campaign.

## The four sources (each empirically confirmed before fixing)

1. **Thread interleaving** (Tracking ∥ LocalMapping ∥ LoopClosing): fixed by
   `ORB_DETERMINISTIC=1` — no worker threads; queues drained synchronously per frame
   (`ProcessQueueOnce()` extraction; synchronous reset/stop handshakes; inline GBA;
   `cv::setNumThreads(0)`). Bonus: kills the pre-existing in-process g2o race → experiment
   processes can run N-wide in parallel.
2. **Pointer-ordered containers** (`std::set/map<KeyFrame*/MapPoint*>`): iteration order =
   allocator layout; measured NOT reproducible even single-threaded with ASLR off. Fixed with an
   `IdLess` (mnId) comparator across Map/MapPoint/KeyFrame containers + covisibility weight-tie
   sorts (was: ties broken by pointer). Frames 1–385 became bit-identical after this.
3. **Eigen vectorization**: peels loops to heap-pointer alignment → FP summation split varies
   run-to-run. Fixed with `EIGEN_DONT_VECTORIZE` on ORB_SLAM3 + bundled g2o (ABI-safe).
4. **An uninitialized heap read in post-IMU-init tracking** (upstream bug, still open):
   proven by `MALLOC_PERTURB_=42` — with heap garbage pinned to a constant, two full runs are
   **bit-identical (same trajectory md5)**; with natural garbage, runs diverge starting at the
   first tracked frames after `InitializeIMU` (bisect trail: extraction, mono inits, tracking
   poses, InertialOptimization outputs, FIBA chi2 — all bit-identical to 17 digits; KF velocities
   at the NEXT init differ). With perturb the poisoned value (0x42-pattern ≈ huge floats) degrades
   quality catastrophically — so the read demonstrably feeds estimator state, and in normal runs
   its random garbage is the residual jitter. Source fix pending (localized to the first
   post-init tracking frames; `ORB_DET_DEBUG` instrumentation left in place for the hunt).

## Where determinism stands

| mode | 542 gap draws | coverage | bit-exact? |
|---|---|---|---|
| threaded (stock arch) | 3.4 – 53 m | lottery incl. resets | no |
| **sequential deterministic** | **2.54 – 3.18 m (8 draws)** | **1201/1201 every draw** | not yet (source-4 jitter) |
| sequential + MALLOC_PERTURB_=42 | deterministic but poisoned | — | **yes (md5-identical)** |

Sequential mode is also *better*, not just tighter: every draw beats the best-ever threaded draw
(3.42 m), because LocalMapping processes every KF (no busy-drops).

## Infrastructure

- `eval/det_queue.sh` — deterministic-mode batch runner (N parallel processes, setarch -R,
  per-run gap/vATE/md5 into `runs/det_campaign_summary.csv`).
- Fork commit `24c0738` (branch `nora-altitude-fusion`).
- 10×542 + 10×538 + 3×circle measurement batch in flight.

## circle_move (user timestamp 1775119067080676352)

The existing conversion covered only pre-flight ground time (IMU-activity analysis: the actual
flight is the FINAL ~11 s of the session, exactly from the user's stamp). Rebuilt
`dataset/circle_move` from t−2 s to session end (270 frames, 14 s, IMU 372 Hz; old junk segment
preserved as `circle_move_prelight_junk`). Stock inertial-init timing (2 s + VIBA1@5 s +
VIBA2@15 s) barely fits a 14-s flight — threaded probes all died pre-init; deterministic-mode
probes in the current batch.
