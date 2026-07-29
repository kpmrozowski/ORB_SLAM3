# Loop detection unblocked — the three-lock chain, found and opened

**Date:** 2026-07-16 ~14:30. Endpoint-gap campaign, part 2 (part 1: 20260716-1300).

## The debug chain (ORB_PR_DEBUG, flight 542 + the 13-loop 538 draw)

Loop detection wasn't rare because of missing candidates — it was rare because THREE independent
locks sat one behind another:

1. **BoW candidates exist all flight long** (`loopCand=1` on dozens of KFs; 103–137 BoW matches
   per candidate vs threshold 12). The place-recognition front-end was never the problem.
2. **Lock 1 — Sim3-RANSAC aborts before iterating**: the solver only accepts a 3D-3D
   correspondence when the current KF *also* has its own map point at the matched keypoint.
   Measured usable N = **4–15 (median 8)** of those 100+ matches (nf5000 keypoints, ~10× fewer
   mapped) vs stock minInliers 15 → `bNoMore` with 0 inliers, every candidate, every run — the
   zero-detection mechanism across ~20 runs. Fix: `ORB_PR_SIM3_MININL` (commit 451001c), set 4.
3. **Lock 2 — the yaw tolerance**: the one draw that did detect (f538_fused_r1, 13 detections)
   was rejected 13/13 — measured `phi` shows **consistent yaw 0.38–0.41 rad (~22°)** across all
   candidates = the map's true accumulated yaw drift, *above* the stock 0.349 yaw gate. Relaxing
   roll/pitch alone (yesterday's exp) could never accept these. Fix: `ORB_LOOP_YAW_TOL=0.60`.
4. **Lock 3 — roll/pitch gate** (0.46°, vs 3–15° gravity-alignment scatter): already relaxed
   (`ORB_LOOP_RP_TOL=0.20`), safe since the unconditional 4-DoF lock discards roll/pitch anyway.

## Fused-altitude arms (batch pr3, first valid runs after the `#`-header loader fix)

- **538: fused reference works** — f538_fused_r1 vATE **3.89 m / v-scale 0.993** (best 538 vATE
  to date), r2 10.42/1.054. (r3 = catastrophic-mode draw.)
- **542: fused-v1 backfired** (gaps 17–109 vs baro-twin ~3–44): with the 10/80 trust curve the
  ~24 m cruise sits at w≈0.8 rangefinder, and crop/hedge AGL texture (~2 m) becomes a 30σ
  z-distortion under σ0.075 edges. Fix: **terrain-texture low-pass** — only the slow component
  (5 s centered mean) of the rfnd correction passes at cruise; the fast component (incl. the
  −6.9 m baro landing dip) passes only below ~12 m AGL where terrain = home spot. User's 10/80
  trust weighting kept as the amplitude envelope.

## Gap-metric caveat (learned from f538_fused_r1's fake 892 m)

The endpoint gap is only meaningful when the FIRST map survives to landing — after a map reset
the TUM file continues in a new frame. Runs with resets report garbage gaps; the summary CSV
keeps them but analysis must filter (poses≈coverage + no "Creation of new map" in log).

## In flight (batch pr4, guarded runner v5)

Full stack on both flights ×3: fused(texture-LP) + σ0.075 + f0.5 + RP_TOL 0.20 + YAW_TOL 0.60 +
NCAND 8 + COINC 2 + MATCH_SCALE 0.6 + SIM3_MININL 4 + 4-DoF lock + PRDBG. Also new: machine-wide
MI guard in the queue runner (the user runs interactive sessions — 4 concurrent MI processes
spotted and defused; ≤2 total enforced with random stagger).
