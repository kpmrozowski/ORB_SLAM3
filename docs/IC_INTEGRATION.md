# IC (inverse-compositional) ORB false-positive match filter

Env-gated, fail-open dense photometric refinement that prunes ORB false-positive matches at
monocular initialization and at frame-rate tracking. Ported from the radiolus alignment lab
(`corr::Correlation` / `refine_homography`) and upgraded with the lab-proven best-of-seed family,
ensemble ECC voting, and the relative acceptance gate. **Defaults preserve stock ORB-SLAM3 behavior
byte-for-byte** — nothing runs unless a flag is set, and any missing ingredient leaves matches
untouched.

## What it does

* **Seed (`ICSeed`)** — an IMU/barometer-only homography `H21 = K (R21 + t21 nᵀ/d) K⁻¹` in the
  undistorted `P == K` pixel domain. `R21` is always gyro preintegration; the translation/plane use
  the barometer climb rate, a forward-speed prior, and a gravity + AGL ground plane. **Never seeded
  from ORB.** A candidate family (full seed, rotation-only `K R21 K⁻¹`, and — TWMM only — a
  previous-pair propagation) is scored by consistent ZNCC on the undistorted pair and the best wins.
* **Refine (`ICEngine`)** — inverse-compositional dense alignment at full resolution. The reductions
  use a fixed-64-chunk `std::thread` summation, so results are **bit-identical for any thread count**.
* **Init filter (`ORB_IC_INIT`)** — refines the seed against the retained init reference image, then
  prunes `mvIniMatches` whose symmetric transfer error vs the reference H exceeds `ORB_IC_FILTER_PX`.
  A survivor guard (`≥ ORB_MINIT_MINMATCHES`) means the filter can never starve initialization.
  `TwoViewReconstruction` still consumes only the survivors — no TVR surgery.
* **Track gate (`ORB_IC_TRACK`)** — in `TrackWithMotionModel`, before the `nmatches < 20` check,
  prunes current-frame map points whose last-frame pixel transfers more than `ORB_IC_TRACK_PX`.
  Active pre-IMU-init / post-reloc (TWMM early-returns once IMU is initialized). Survivor floors
  (20 matched / 10 map-observed) mean the filter can never fail TWMM on its own.
* **Ensemble ECC voting (`ORB_IC_ENSEMBLE`)** — at the init filter only, also fit
  `H_orb = findHomography(RANSAC, 3.0)` on the matched undistorted points and use whichever of
  `{H_ic, H_orb}` scores the higher consistent ZNCC **as the filter reference**. Voting only picks the
  gate reference; ORB never seeds IC and TVR is untouched.

## THE CASCADE (`ORB_IC_CASCADE`) — the alignment stack used *constructively*

The overnight campaign falsified match REJECTION at 5 fps (all-IC-on regressed vs baseline; map-recycling
after divergence is now forbidden — abort + report only). The winning lever must therefore be **better
tracking**, so the cascade turns the IMU→IC→ORB stack into a *prior* instead of a *rejector*. Per tracked
pair (TWMM) and per init pair, all under `ORB_IC_CASCADE=1` (fail-open, stock-preserving when off):

1. **IMU seed family** (`ICSeed`) → best-ZNCC `H_seed` (`ecc_seed`).
2. **IC refine** (`ICEngine`) → `H_ic`, `ecc_ic`.
3. **LAZY ORB escalation** — only if `ecc_ic < ORB_IC_ESCALATE_ECC` (or IC failed): BF+Lowe descriptor
   matches (init: the existing `mvIniMatches`; track: both frames' descriptors) → RANSAC `findHomography`
   on the **undistorted** points → `H_orb`, `ecc_orb`. Escalation cost is paid only when IC is weak, so the
   escalation rate is the instrument (`escalated` column): day flights should stay low, dusk flights higher.
4. **Winner** `H_best = argmax ecc(H_ic, H_orb)`, accepted only if it clears the tolerance gate (else the
   pair yields nothing — fail open).

**Constructive consumption of `H_best`** (each sub-flag independently tunable):

* **`ORB_IC_CASCADE_PRIOR`** — in `TrackWithMotionModel`, pre-IMU-init, where TWMM otherwise predicts the
  pose from the stale constant-velocity `mVelocity`. We decompose `H_best` against the seed's ground plane
  and use the result as the prediction. **Algebra:** a planar homography in the undistorted `P==K` pixel
  domain is `H = K (R21 + t21·n1ᵀ/d) K⁻¹`, so `M = K⁻¹ H K = R21 + (t21/d)·n1ᵀ`. `R21` (gyro) and the unit
  plane normal `n1` (gravity, ref-cam) are both KNOWN from the IMU seed, so the rank-1 term is isolated and,
  since `n1` is unit, right-multiplying by `n1` recovers the translation directly: `(M − R21)·n1 = t21/d`.
  The prior applies `R21` as the rotation and, when `|t21|/d > ORB_IC_PRIOR_MIN_TRANS`, rotates the *map-scale*
  `mVelocity` translation onto the `t21` direction (the homography fixes rotation + translation direction;
  map scale stays with `mVelocity`, the only metric-consistent magnitude pre-IMU-init). `SetPose(T_cur_ref ·
  T_last)` with `T_cur_ref = (R21, t_pred)`.
* **`ORB_IC_CASCADE_GUIDE`** — after the stock `SearchByProjection`, a purely ADDITIVE pass: each
  still-unmatched last-frame map point's pixel is warped through `H_best` (undistort → H → re-project via the
  camera model into the distorted image), a window of `ORB_IC_GUIDE_RADIUS` is searched, and an empty current
  keypoint is filled. It never overwrites or removes an existing match.
* **`ORB_IC_CASCADE_RESCUE`** — on a pre-IMU-init `RECENTLY_LOST` frame (before full relocalization), the
  same decomposition seeds a pose from the last frame so the normal `TrackLocalMap` gets one re-lock attempt;
  success (state returns to `OK`) is counted. On failure the stock lost path resumes unchanged.
* **`ORB_IC_CASCADE_INITFILTER`** (default OFF) — the old `mvIniMatches` symmetric-transfer prune vs
  `H_best`, with the same `≥ MInitMinMatches` survivor guard. Off because init-time rejection has
  historically destabilized these flights; the tuner may re-test.

**Divergence guard** (`ORB_DIVERGE_*`) stays abort-by-default; the cascade recipes must not set
`ORB_DIVERGE_ACTION`. The `mono_inertial_euroc` example now always prints a `RUN SUMMARY: coverage
N/M poses (P%) [COMPLETE|DIVERGED-ABORT]` line so an abort never silently forfeits a flight without a number.

## Environment variables

| Variable | Default | Meaning |
|----------|---------|---------|
| `ORB_IC_INIT` | unset (off) | Enable the init-pair match filter |
| `ORB_IC_TRACK` | unset (off) | Enable the TWMM frame-rate gate |
| `ORB_IC_ENSEMBLE` | unset (off) | Ensemble ECC voting (init filter reference = argmax ZNCC of H_ic vs H_orb) |
| `ORB_IC_SEED_PREV` | unset (off) | Add the previous-pair propagation seed candidate (TWMM only) |
| `ORB_IC_FILTER_PX` | `3.0` | Init: symmetric transfer gate (px) for match invalidation |
| `ORB_IC_TRACK_PX` | `6.0` | Track: one-way transfer gate (px) for map-point rejection |
| `ORB_IC_MIN_CORR` | `0.8` | Init: absolute acceptance ZNCC (Absolute gate) |
| `ORB_IC_TRACK_MIN_CORR` | `0.75` | Track: absolute acceptance ZNCC (Absolute gate) |
| `ORB_IC_GATE_MODE` | `abs` | `abs` = min-corr; `rel` = floor + tolerance below the used seed |
| `ORB_IC_GATE_FLOOR` | `0.5` | Relative gate: absolute floor the refined ZNCC must clear |
| `ORB_IC_GATE_MARGIN` | `0.05` | Relative gate: tolerance the refined ZNCC may fall below the used seed |
| `ORB_IC_TRACK_MIN` | `20` | Track: minimum matched map points before the gate applies (survivor floor) |
| `ORB_IC_MAX_DT` | `1.0` | Refuse the IMU seed when the pair Δt (s) exceeds this (dropped-frame guard) |
| `ORB_IC_SPEED_MPS` | `0` | Forward-speed prior for the full seed's horizontal translation (m/s) |
| `ORB_IC_RFND_CSV` | unset | Optional rangefinder AGL source (`t_ns,dist_m`); overrides baro AGL when present |
| `ORB_IC_GAUSS_ITERS` / `ORB_IC_GN_ITERS` | `30` / `60` | Init refinement Gauss / Gauss-Newton iteration budgets |
| `ORB_IC_TRACK_GAUSS_ITERS` / `ORB_IC_TRACK_GN_ITERS` | `8` / `12` | Track refinement iteration budgets |
| `ORB_IC_THREADS` | `1` | Worker threads for the chunked reductions (bit-identical for any value) |
| `ORB_IC_DEBUG` | unset | Stderr `IC_INIT` / `IC_TRACK` / `IC_CASC` lines + per-pair CSV |
| `ORB_IC_DEBUG_DIR` | unset | Directory for `ic_stats.csv` (must already exist) |

### The cascade (`ORB_IC_CASCADE`) — constructive consumption

| Variable | Default | Meaning |
|----------|---------|---------|
| `ORB_IC_CASCADE` | unset (off) | Master flag: run the IMU→IC→(lazy ORB) chain per tracked/init pair and consume H_best constructively (replaces the rejection filters — legacy `ORB_IC_TRACK` is bypassed while this is on) |
| `ORB_IC_CASCADE_PRIOR` | unset (off) | Replace the constant-velocity TWMM prediction (pre-IMU-init) with the H_best-decomposed pose |
| `ORB_IC_CASCADE_GUIDE` | unset (off) | ADD matches by re-projecting still-unmatched last-frame map points through H_best (additive, never rejects) |
| `ORB_IC_CASCADE_RESCUE` | unset (off) | On pre-IMU-init RECENTLY_LOST frames, seed a pose from H_best and let TrackLocalMap re-lock once |
| `ORB_IC_CASCADE_INITFILTER` | unset (off) | Old mvIniMatches transfer prune vs H_best at init (OFF: init-time rejection historically destabilizes) |
| `ORB_IC_ESCALATE_ECC` | `0.6` | Compute the lazy ORB rung only when the IC ZNCC is below this (day flights escalate <10 %, dusk more) |
| `ORB_IC_ESCALATE_LOWE` | `0.75` | Lowe ratio for the escalation's BF (Hamming) descriptor matches |
| `ORB_IC_GUIDE_RADIUS` | `-1` | Guided-search window radius (px); `≤0` = the stock `th·scale` window |
| `ORB_IC_PRIOR_MIN_TRANS` | `0.02` | `\|t\|/d` above which the prior takes its translation DIRECTION from H (else it keeps the map-scale velocity translation) |

The cascade gate is **tolerance-form only** — `ecc(H_best) ≥ ORB_IC_GATE_FLOOR (0.5)` **and** `ecc(H_best) ≥ ecc_seed − ORB_IC_GATE_MARGIN (0.05)`. The absolute `ORB_IC_MIN_CORR` (0.8) gate is **not** applied on the cascade path (it stays in force only for the legacy non-cascade `ORB_IC_INIT`/`ORB_IC_TRACK` filters).

Related existing envs the seed reads: `ORB_BARO_CSV` (barometer series → climb rate + AGL),
`ORB_DETERMINISTIC` (context; the reductions are deterministic regardless).

## `ic_stats.csv` columns (v2)

`kind` (INIT/TRACK/**CASC**), `frame_ref`, `frame_cur`, `t_ref`, `t_cur`, `n_matches`, `seed_src`
(rot/full/prev/failed), `used_seed_zncc`, `ecc_seed_rot`, `ecc_seed_full`, `ecc_ic`, `ecc_orb`
(-1 when not computed), `ensemble_winner` (ic/orb/none — for CASC this is the cascade **winner**),
`gate` (INIT/TRACK: pass/lowcorr/guard; CASC: pass/floor), `invalidated`, `survivors`, `ms`,
**`escalated`** (0/1 — lazy ORB rung ran), **`consumed`** (letters: `p`=prior `g`=guide
`r`=rescue-attempt `R`=rescue-success `f`=init-filter, `-` = none).

The two `escalated`/`consumed` columns are **appended** last, so column-name parsers stay backward
compatible; legacy INIT/TRACK rows write `0` / `-`. One CASC row is emitted per tracked/init/rescue frame,
flushed after the frame's state is resolved (so `r`→`R` reflects the true rescue outcome).

## GPS[0] overlay (`ORB_GPS_CSV`)

Separate viewer-only feature (`GpsOverlay` / `MapDrawer::DrawGPS`). Reads `t_rebased_ns,e,n,u,valid`,
aligns valid GPS to keyframes with a free `Eigen::umeyama` similarity (refit ~2 s), and draws a yellow
trajectory line-strip, dark-yellow sample points, and bright-orange keyframe↔GPS correspondence lines.
Two Pangolin checkboxes ("GPS[0] traj" / "GPS↔SLAM lines"). Inactive without `ORB_GPS_CSV`.

## Scope notes

* No in-SLAM flipbook PNG dumping — the radiolus lab owns visual debug; `ic_stats.csv` + the stderr
  lines are the tuning surface here.
* The map-point RANSAC plane rung is intentionally not built: it is only reachable after a metric map,
  i.e. after IMU init, where the TWMM gate early-returns and the seed is never requested. The gravity +
  baro/rfnd AGL plane (metric for both the init and pre-init-track windows) is the only plane path that
  ever executes.
