# 07 — Things Tried That Did Not Improve (or Caused Divergence)

An honest record of the experiments that failed, regressed, or diverged. Several of these shaped the final design by *elimination* — knowing what does not work is as valuable as the wins in [doc 06](06-vanilla-algo-all-good-mods.md).

---

## Resolution, calibration, and the IMU model

**Full-resolution native-1640 VIO regressed every estimator.** Re-running everything at the native 1640×1232 resolution with the native fisheye calibration (instead of the 800-px downscale) was expected to help; instead it regressed uniformly. On flight 538 the monocular-inertial estimator saved only a 15 %-coverage fragment or nothing at all; mono-only managed 18 %. The 800-px baseline (which tracks 92 % of 538) was kept. The root cause is not resolution or calibration — it is **weak high-altitude inertial observability** (little parallax, weak accelerometer excitation), which more pixels cannot fix.

**"Measured" Allan-variance IMU noise was strictly worse than the legacy model.** Replacing the hand-tuned noise parameters with Allan-variance-measured values (a 2-hour static log) *collapsed* matching: on flight 538 the fraction of frames with fewer than 30 map matches jumped from 5 % to 90 %. Tighter noise makes the estimator over-trust an imperfect IMU and its extrinsics. The legacy noise model wins. (An untested idea remains: inflate the measured noise ×2–5 to account for in-flight vibration and thermal drift.)

**VINS-Fusion diverges on this data.** As an external cross-check, VINS-Fusion diverged on essentially every full-resolution moving-start sequence (538 ran away to ~889 km, 542 to ~42 km). Because no non-diverged VINS run ever existed, the planned compass/mag fusion on top of it was never even applicable.

## Feature selection

**A sky / brightness detection mask hurt SLAM.** Masking out the bright sky (to stop features on clouds) sounds sensible but backfired: a narrow circular mask halved flight 538's correct matches, killed flight 542's tracking entirely (19 poses), and collapsed 538's vertical scale from 0.305 to 0.072. At 130 m AGL the *peripheral* features carry the altitude parallax, so removing them removes the very signal that scales the map. Masks are viewer-only; the follow-up idea is an attitude-aware horizon filter rather than a static circle.

**nFeatures = 1500 is an initialization lottery, not a setting.** A run at nf1500 once produced an impressive vertical ATE of 1.49 m — but it was a lucky no-baro draw; re-draws scattered 1.79–9.93 m, and adding tight barometer fusion killed 3 of 5 maps outright. Low feature counts make initialization a gamble. The reliable operating point is nf5000 + CLAHE.

## Barometer fusion — designs that were rejected

Several barometer-edge designs were tried and rejected *with evidence* before the final relative-datum + insertion-gate design was settled:

- A **stored absolute per-map datum** — flight 538 exploded to kilometre scale.
- A **window-median-only datum** — drift-preserving; the anchor itself drifted ~5 m per 100 s.
- A **Huber robust kernel** on the baro edges — it saturates, and vertical scale got stuck at 0.4.
- **Periodic scale refinement layered on top of the z-edges** — the two mechanisms fought each other and the scale thrashed.
- **σ = 0.05** — too stiff; the fit floors out (see the σ ladder in doc 06).
- A per-frame edge **σ tighter or looser than 0.5** — both harmful.

## Rangefinder and fused altitude

**Rangefinder fusion regresses over relief terrain.** On flat ground the rangefinder beats the barometer (doc 06 §C), but over the rising/falling terrain of flight 538 it collapses the vertical scale (0.71 → 0.54), because it measures height above the *terrain*, which is not a fixed vertical datum. The tilt correction (`AGL = slant · cos(roll) · cos(pitch)`) matters in magnitude — tilt reaches p50 15–17°, adding ~6 m of slant excess at 538 cruise — but its run-level effect stays within draw noise. A **fused baro+rangefinder reference** was outright *harmful* on 538: the climb-phase tilt correction reshapes the early map and makes the endpoint gap 6× worse. It only helps on flat, in-range flights.

## Heading and horizontal error

**The magnetometer does not shrink the endpoint gap.** Adding mag fusion left flight 542's gap essentially unchanged (9–40 m across draws, vs a 3.4–44 m baseline). The horizontal residual on these flights is monocular **scale drift**, which a yaw constraint cannot touch. (Mag's value is absolute north and map longevity, not horizontal accuracy — see doc 06 §B.)

## Loop closure

**Live loop closure was never achieved on the survey flights.** The detection machinery was fully built and made safe (doc 06 §F), but no true closure fired on 538/542. Two reasons: mid-flight bag-of-words candidates over self-similar farmland are **descriptor aliasing** that RANSAC correctly rejects, and the one genuine landing revisit has too few both-sides-mapped correspondences to verify even with the relaxed floors. The free-scale Sim(3) did not unlock it either. The unconditional 4-DoF lock is correct by construction but has **never been exercised on a live detection**. An earlier "loop-gate composition improves results" claim was **retracted** — those runs actually had zero detections and the apparent effect was recipe noise.

## Initialization gates

**Shortening the initialization gates is counterproductive on long flights.** The configurable gates (doc 06 §G) exist to let genuinely short flights start; but *reducing* them on a long flight hurts — the circle flight closed to 1.07 m with shortened gates vs 0.27 m with the stock schedule. Keep stock gates whenever the flight is long enough to satisfy them.

## Data quality walls

**The "circle final-11s" footage has no usable flight** — 2 seconds of violent motion blur followed by static grass; the endpoint gap is simply unmeasurable from it.

**A ~30 % catastrophic-init rate persists at any tight σ on flight 538.** Vertical scale occasionally collapses to ≤ 0.005 (position explodes) or a full-inertial-BA segfault fires. The per-frame baro edge suppresses this to ~10 % but does not eliminate it — a genuinely robust weak-excitation initializer is still needed.

**The historical fleet is mostly un-scoreable and often diverges.** Of 52 datasets, 36 diverge in the climb (velocity blow-up), and GPS-instance-0 is spoofed from takeoff on most, leaving only ~6 flights scoreable against GPS. Some fleets have unreliable compasses (papa3). And the rescue knobs cut both ways: `ORB_MAG_SIGMA_DEG=10`, which saved five flights, *regressed* a few others (282_golem17 coverage 22 % → 6 %, 52_golem28 35 % → 23 %).

## Determinism

**One nondeterminism source remains unfixed.** Three of four causes were eliminated (doc 06 §H), but an **uninitialized heap read** just after IMU init persists — flight 538 still flips between two trajectories about 1 in 10 runs. It is localized (a `MALLOC_PERTURB_` probe flips the md5) but the offending read has not yet been pinned down in source.

---

**Bottom line.** The two open walls behind most of these failures are **monocular horizontal scale drift** (only loop closure or an absolute horizontal anchor removes it — neither barometer nor magnetometer can) and the **catastrophic-initialization lottery at altitude** (halved by the per-frame baro edge, not yet eliminated). See [`../DEVELOPMENT_REPORT.md` §7](../DEVELOPMENT_REPORT.md#7-open-problems--future-work) for the forward plan.
