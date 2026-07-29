# circle_move: init gates shortened via env — and the ground truth about the recording

**Date:** 2026-07-22. User ask: shorten the inertial-init gates via env and rerun circle_move.

## New env knobs (commit 8d8268f, defaults preserve stock exactly)

| env | stock | role |
|---|---|---|
| `ORB_IMU_INIT_MINTIME` / `ORB_IMU_INIT_MINKF` | 2 s / 10 KF | first inertial init |
| `ORB_IMU_VIBA1_S` / `ORB_IMU_VIBA2_S` | 5 s / 15 s | VIBA marks (low-motion reset window scales with VIBA2) |
| `ORB_TRACK_MININL_PREIMU` | 50 | TrackLocalMap inlier floor before IMU init |
| `ORB_TVR_SIGMA` / `ORB_TVR_MINPARALLAX` / `ORB_TVR_GOODFRAC` | 1 px / 1° / 0.9 | two-view reconstruction acceptance |
| `ORB_MINIT_MINMATCHES` / `ORB_MINIT_WINDOW_PX` | 100 / 100 px | mono-init match gate / search window |

Each layer was found empirically (deterministic mode + `ORB_DET_DEBUG`/DETMINIT bisect):
stock reconstruction failed **199/199** attempts; σ3/0.3°/40% initialized twice during takeoff;
the fresh maps then died within 0.2 s; in-flight init attempts stopped at the 100-match gate.
Also learned: pre-init frames carry ~5× nFeatures keypoints by design (the init extractor) —
the "22k keypoints" observation was not a bug.

## Dataset findings

1. `circle_move` rebuilt earlier with a 2 s lead-in initialized on **ground vibration** (t=1.48,
   pre-takeoff) — garbage parallax. Reconverted as `circle_move2` starting exactly at the user's
   stamp 1775119067080676352.
2. **The decisive evidence — the recording contains no circle flight.** Inter-frame image
   difference over the full clip: real motion only **t = 0–2.3 s** (violent, heavily
   motion-blurred hop; ~45 frames), then **t = 2.5–11.9 s completely static** (MAD 0.36–0.46
   grey levels = sensor noise) with the camera parked half-buried in grass. The gyro activity
   (0.48 rad/s RMS) in this window is vibration, not maneuvering. Sample frames:
   scratchpad circ_frame020/100/150.png — t≈4.7 s and t≈7.2 s are visually identical.

## Verdict

The endpoint gap of circle_move **cannot be verified by any VIO from this footage**: the only
moving segment is ~2 s of extreme blur (too short/violent even for the shortest init schedule),
and the remaining 80% of the clip has zero parallax plus non-rigid grass in prop wash. The gate
envs are validated and remain valuable for genuinely short flights; for the circle target we
need a re-flown recording (higher altitude, gentler start, or higher fps/shorter exposure).
