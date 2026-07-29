# 06 — The Algorithm and the Modifications That Worked

This is the deep-dive: a short primer on how vanilla ORB-SLAM3 monocular-inertial works, then every modification on this branch that **improved** results, with its mechanism, its environment knobs, and the measured effect. (Things that did *not* work are in [doc 07](07-tested-not-improving-mods.md).)

---

## Part 1 — Vanilla ORB-SLAM3 mono-inertial in brief

ORB-SLAM3 runs three cooperating threads over an **Atlas** of maps:

1. **Tracking** extracts ORB features from each frame, matches them to the local map, and estimates the camera pose by minimizing reprojection error. In the inertial mode it also fuses IMU preintegration between frames. When tracking is confident it spawns a **keyframe**.
2. **Local Mapping** triangulates new map points from keyframes, runs local bundle adjustment (BA) over a covisibility window, and culls redundant keyframes/points. It also drives **inertial initialization**: a coarse IMU init, then two refinements, **VIBA1** (~5 s) and **VIBA2** (~15 s), which recover gravity direction, velocity, biases and — for monocular — the **metric scale**.
3. **Loop Closing** detects revisited places via a bag-of-words (DBoW2) database, verifies them geometrically with a Sim(3) solver, and corrects accumulated drift with a pose-graph optimization and a global BA.

**Why this struggles on high-altitude aerial cruise.** Monocular scale is only observable from IMU acceleration, and at cruise the accelerometer excitation is weak and parallax is tiny. So inertial init is fragile (frequent map resets, or scale collapsing to near-zero and the position exploding), and once initialized the monocular **scale drifts** along the flight. The modifications below attack exactly these failure modes.

Everything is **env-gated and default-off**: with no `ORB_*` set the binary is stock ORB-SLAM3.

---

## Part 2 — Modifications that improved results

### A. Barometer tight fusion — the biggest win

A barometer gives a direct, drift-free (if noisy) measurement of the one thing monocular VIO cannot pin down at altitude: the vertical channel. The fork adds three g2o edge types (`include/BaroFusion.h`) and wires them into the optimizers:

- **`EdgeBaroZ`** — a unary constraint on each keyframe's world-z equal to the barometer altitude, with an **analytic Jacobian**. It is added inside local and full inertial BA against a **gauge-safe relative datum** (the z−baro offset of the newest fixed keyframe, recomputed every optimization so it survives global scale/rotation updates). It is **quadratic with no robust kernel** — the barometer has no outliers — but each edge passes through an **insertion gate** (`ORB_BARO_GATE`, metres) that skips a keyframe whose current inconsistency is large (a relocalization glitch, not drift).
- **`EdgeBaroScaleGDir`** — an **initialization-time** edge on the gravity-direction and scale vertices. With the poses fixed, the barometer's vertical displacement directly informs `(gravity, scale)`, so **the map is born metrically-scaled** even when the accelerometer barely moved. This is the mechanism that makes scale observable at cruise.
- **Per-frame baro edge** (`AddFrameBaroEdge`, gated by `ORB_BARO_FRAME_SIGMA`) — the same z pin, but added in the tracking-thread pose optimizers at frame rate, with a causal relative datum. It carries the fragile pre-VIBA2 phase through initialization without the reset-thrash that otherwise wrecks the first seconds of a high flight.
- A periodic **scale refinement** in Local Mapping (`ORB_BARO_SCALEREF_S`) re-runs the `(gravity, scale)` optimization every N seconds so vertical scale cannot drift after upstream's one-off refinement.

**Knobs:** `ORB_BARO_CSV` (enables), `ORB_BARO_SIGMA` (1σ in metres, the key tuning dial), `ORB_BARO_GATE`, `ORB_BARO_FRAME_SIGMA`, `ORB_BARO_SCALEREF_S`.

**Measured effect (flight 542, low & flat; vehicle EKF scores 1.42 m vertical ATE on the same span):**

| `ORB_BARO_SIGMA` | vertical ATE | drift RMS |
|---|---|---|
| 1.0 | 5.37 m | 2.52 m |
| 0.15 | 1.28 m | 1.88 m |
| **0.075** | **1.15 m** | 1.53 m |
| 0.05 | 1.70 m (too stiff) | 1.64 m |

At σ=0.075 the fork **beats the vehicle's own EKF** (1.15 m vs 1.42 m). The clean physics result behind the ladder is the **fusion-lag law**: `EdgeBaroZ` reproduces the barometer *late*, with lag proportional to σ (σ1.0 → ~1.4 s, σ0.3 → ~0.5 s). On fast climbs this lag *is* most of the vertical error, so lowering σ tightens the fit — until it becomes too stiff and floors out. On the hard flight 538 (high, over relief), σ0.075 cut vertical ATE from 20.2 m to a median of ~7 m and repaired vertical scale from 0.71 to ≈1.0; adding the per-frame edge (`ORB_BARO_FRAME_SIGMA=0.5`) cut the catastrophic-init rate from ~37% to ~10%.

### B. Magnetometer fusion — absolute heading

`include/MagFusion.h` adds **`EdgeMagYaw`**, a unary yaw constraint from the Earth field pre-rotated into the body frame (`h = R·m_body`, residual on `atan2(h.y, h.x) − yaw_ref`), plus a **compass-aligned initialization** that rotates the world about gravity so map yaw equals magnetic north. Both are gated on a mature init (VIBA2) — a strong yaw edge on a coarse early attitude corrupts short flights into reset loops.

**Knobs:** `ORB_MAG_CSV` (enables), `ORB_MAG_SIGMA_DEG` (1σ in degrees).

**Effect:** accuracy gain is *marginal* (538 horizontal RMSE 292 → 287 m) because the residual horizontal error is scale drift, not yaw. The real value is elsewhere: an **absolute-north** map for GPS-denied navigation, confirmation that VIO attitude is genuinely good (heading drift bounded ±20° over 9 min), and **map longevity** (3× fewer resets). Across the historical fleet, `ORB_MAG_SIGMA_DEG=10` rescued **five flights from divergence** — e.g. 243_golem17 coverage 31 % → 99.4 % with the endpoint gap collapsing 2034 m → 39 m, and 357_golem17 53.5 % → 97.2 %, gap 1800 m → 16 m.

### C. Rangefinder as an altitude source

`BaroFusion` is altitude-source-agnostic, so pointing `ORB_BARO_CSV` at a rangefinder CSV fuses **AGL** instead of barometric altitude. On flat terrain this wins (flight 542 vertical ATE 4.45 m vs baro's 5.37 m); over relief it *loses* (see doc 07) because the rangefinder measures height above the *terrain*, not a fixed datum. A tilt correction (`AGL = slant · cos(roll) · cos(pitch)`) and a fused baro+rangefinder reference (rangefinder low, barometer high) can remove the barometer's landing dip on flat flights.

### D. Front-end extraction — CLAHE is load-bearing

Aerial footage is often low-contrast. Applying **CLAHE** (contrast-limited adaptive histogram equalization) before ORB extraction, together with a high feature cap, is decisive: on flight 542 the median count of RANSAC-verified correct matches per frame-pair rose from 21 to 1364; on the circle flight, closure went from a **200 m gap without CLAHE to 0.27 m with it**. CLAHE is applied identically in the tracker and the prefetcher so results are consistent.

**Knobs:** `ORB_CLAHE` (clip limit, e.g. 3.0), `ORB_CLAHE_TILE` (tile grid).

### E. Feature prefetch — offline speed, zero result change

ORB extraction depends only on the image and extractor parameters, so it is precomputed off the tracking thread by a worker pool (`FeaturePrefetcher`) with per-worker extractor and CLAHE clones that mirror the tracker exactly. It is bit-identical to inline extraction and ~2.4× faster for offline runs. **Knobs:** `ORB_PREFETCH=N|all`, `ORB_NO_PACE=1`.

### F. Place-recognition / loop-closing robustness

The canonical landing-over-takeoff loop never fired under stock thresholds. Instrumentation (`ORB_PR_DEBUG`) localized the death: bag-of-words candidates were plentiful (100+ matches), but the **Sim(3) RANSAC** starved — only 4–15 correspondences survive the "both sides must be mapped" filter, below the stock minimum of 15, so RANSAC aborted before a single iteration. The fixes:

- **`ORB_PR_SIM3_MININL`** — a configurable Sim(3) minimum-inlier floor (set to ~4 to admit the sparse-but-real revisits).
- **`ORB_PR_FREE_SCALE`** — allow a free-scale Sim(3) even after VIBA2, because monocular scale differs at the two ends of a long loop so a fixed-scale hypothesis fits nothing.
- **`ORB_PR_NCAND` / `ORB_PR_COINC` / `ORB_PR_MATCH_SCALE`** — tune candidate counts and match thresholds.
- **Unconditional 4-DoF loop lock** — on *any* accepted inertial loop, force roll = pitch = 0 and scale = 1 (keep yaw + translation), because gravity is observable in any inertial map and a loop must never tilt the world off vertical. This is the **safety net** that makes the relaxations above safe: even a loosely-verified loop can only rotate about gravity.

The machinery is correct and safe; note that a *live* closure on the survey flights was still not achieved (doc 07 explains why — self-similar farmland aliasing).

### G. Initialization gating — let hard flights start

Stock ORB-SLAM3 hard-codes its initialization gates. Short or aggressive flights die before they can initialize. These commits make the gates env-overridable (defaults reproduce stock exactly):

- Inertial-init schedule: `ORB_IMU_INIT_MINTIME`, `ORB_IMU_INIT_MINKF`, `ORB_IMU_VIBA1_S`, `ORB_IMU_VIBA2_S`.
- Pre-init tracking floor: `ORB_TRACK_MININL_PREIMU` (lets a fast-rotation flight limp through the first seconds instead of resetting).
- Two-view reconstruction: `ORB_TVR_SIGMA`, `ORB_TVR_MINPARALLAX`, `ORB_TVR_GOODFRAC`.
- Monocular-init gates: `ORB_MINIT_MINMATCHES`, `ORB_MINIT_WINDOW_PX`.

(Caveat: *shortening* the gates helps only genuinely short flights — on long flights, stock gates are better. See doc 07.)

### H. Deterministic mode — reproducible experiments

Threaded ORB-SLAM3 gives different trajectories every run (flight 542 endpoint gap spread 3.4 → 53 m). Four causes were found and addressed:

1. **Thread interleaving** → `ORB_DETERMINISTIC=1` runs Local Mapping and Loop Closing **synchronously** after each frame (no worker threads), with inline global BA and synchronous reset/stop handshakes. This also removes the in-process g2o race, so multiple **processes** can run in parallel safely.
2. **Pointer-ordered containers** → an `IdLess` comparator orders every pointer-keyed set/map by keyframe/map-point id, so iteration (and thus the floating-point summation order fed to g2o) no longer depends on heap addresses. Always on; semantics unchanged.
3. **Eigen vectorization peeling** → `EIGEN_DONT_VECTORIZE` (compile flag) removes alignment-dependent reduction splits.
4. **An uninitialized heap read** just after IMU init — identified (a `MALLOC_PERTURB_` probe flips the result) but not yet source-fixed; it is the one remaining last-bit jitter.

**Effect:** flight 542's endpoint-gap band collapsed to 2.54–3.18 m and is bit-reproducible; the fresh circle flight closed to **0.27 m, identical across 3 runs**. `ORB_DET_DEBUG` prints per-frame fingerprints for bisecting any residual divergence.

### I. Early-divergence guard — cheap batch campaigns

`ORB_DIVERGE_VMAX` / `ORB_DIVERGE_COUNT`: after IMU init, N consecutive keyframes with velocity above the threshold flag divergence; the run stops and saves the partial trajectory. This turned ~40-minute doomed runs into ~2–5 minutes and revealed that the dominant historical failure is a **velocity blow-up in the first 1–2 minutes** (a catastrophic init in the climb), not gradual drift — 36 of 52 historical runs abort this way.

### J. Preprocessing & correctness fixes

- **`ORB_TSJUMP_S`** raises the frame-gap reset threshold so datasets with multi-second camera drops but continuous IMU (aerial logs) don't fragment the map at every drop — the IMU preintegrates across the gap.
- **`ORB_MASK_FRAC` / `ORB_MASK_FILE`** apply a keep-mask (sky filter) — useful for visualization, but see doc 07 for why aggressive masking hurts SLAM.
- **`ORB_STATS_CSV`** logs per-frame diagnostics (detections, matches, thresholds).
- **Correctness fixes** (always on, no knob): an uninitialized `pBiggerMap` that segfaulted at shutdown on empty maps, and a bracket bug that ran full inertial BA twice.

---

For the full quantitative tables, dataset details, and metric definitions behind these numbers, see [`../DEVELOPMENT_REPORT.md`](../DEVELOPMENT_REPORT.md). For the complete knob list, [doc 08](08-environment-variable-reference.md).
