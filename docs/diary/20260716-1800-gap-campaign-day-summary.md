# Endpoint-gap campaign — day summary (2026-07-16 12:00–18:00)

Targets: takeoff→landing gap **<5 m (538) / <2 m (542)**. Best-known: **3.42 m (542,
night draw), 61.8 m (538)**. NORA EKF's own gaps: 46.3 m (542) / 84.7 m (538) — spoof-GPS
flights, so every healthy SLAM run already closes better than the onboard EKF.

## Where the targets stand

- **542**: the gap is dominated by the H-scale draw lottery. Same-recipe draws span 3.4–53 m
  (today's base542: 6.6/42/53; night's three: 3.4/3.9/5.9). Best draws decompose to ~2.7 m
  horizontal + ~1-2 m vertical. The vertical is solved in principle (fused reference; dz −0.15
  in a v-scale-1.0 draw; tilt-corrected reference draws 6.30 with dz 1.49). The horizontal ~2.7 m
  floor persists across mag (negative), loop stack (no true loops verifiable — see below), and
  reference variants. **<2 m needs functioning loop closure or an H-scale fix; not reached.**
- **538**: healthy-draw gaps 62–133 m, all horizontal drift. The only lever is the
  landing-over-takeoff loop; detection fires in ~1 of 10 draws (map-quality-gated), and in the
  one draw that verified (13 loops, stock thresholds) the loops were rejected for consistent
  yaw 0.38–0.41 rad (> stock 0.349 tol). With today's relaxed gates + 4-DoF lock that draw
  WOULD close — but it did not reappear in 11 harvest draws. **<5 m needs that draw + accept;
  machinery is now in place, the lottery is not conquered.**

## Root causes established (each with data)

1. **Sim3 verification starvation**: only N=4–15 (median 8) of 100+ BoW matches survive the
   both-sides-mapped filter (nf5000, ~10% mapped keypoints) vs stock RANSAC minimum 15 →
   zero iterations, zero detections. Fixed via `ORB_PR_SIM3_MININL` — necessary, not sufficient.
2. **542 candidate aliasing**: with free-scale Sim3 (7 DoF) and N=24–38, bestInliers still 1–3 —
   the mid-flight BoW candidates on self-similar farmland are wrong, and RANSAC is right to
   reject them. The true revisit (KF4/KF5 at landing) has N=1–7 — unverifiable.
3. **538 detection ⟺ good-map draw**: ~700 candidate KFs and 1500–1800 Sim3 attempts per run
   cap at bestInl≈3-4 on ordinary draws; the vATE-3.89/v-scale-0.993 draw verified 13. Loop
   closure quality-gates itself on exactly the maps that need it least.
4. **Rangefinder tilt (user spec)**: median 15–17°, p90 23–26° — slant excess up to ~6 m at 538
   cruise. `ref_att.csv` + `AGL = dist·cos(roll)·cos(pitch)` now in the fusion pipeline
   (`ref_fused_tilt.csv`). Run-level effect on 542 within draw noise (3v3: 6.30/49/163-invalid
   vs 6.58/42/53).

## New C++ (branch `nora-altitude-fusion`, all env-gated, defaults stock)

`ORB_PR_NCAND`, `ORB_PR_COINC`, `ORB_PR_MATCH_SCALE` (a18ca5a); `ORB_PR_DEBUG` cascade
instrumentation (87e18ea, fa4b625); `ORB_PR_SIM3_MININL` (451001c); `ORB_PR_FREE_SCALE`
(4b73997); unconditional 4-DoF accept lock (333d23a); `ORB_BARO_FRAME_SIGMA` (1dd3881).

## New tooling/data

`eval/endpoint_gap.py` (gap metric; valid only for reset-free runs), gap columns + loop counts
in the queue-runner (`night_queue4/5.sh`, machine-wide MI guard after a 4-process incident),
`eval/make_fused_alt.py` (baro+rfnd fusion: 10/80 user trust curve, pre-motor datum,
correction-hold, terrain-texture low-pass, tilt correction), `eval/export_att_csv.py`,
`dataset/*/ref_att.csv`, `ref_fused.csv`, `ref_fused_tilt.csv`.
`runs/gap_campaign_summary.csv` = full per-run record (34 valid + failures).

## What would actually break the two walls (for the next session)

1. **538 <5 m**: make loop verification survive ordinary maps — e.g., verify against the
   candidate's covisibility-window MAP POINTS with projective (PnP-style) matching instead of
   the both-mapped 3D-3D Sim3 (removes the N starvation), or run detection on a low-drift
   sub-map. Alternatively: brute-force draw harvesting with auto-retry on catastrophic init.
2. **542 <2 m**: the horizontal floor is scale drift over 150 s — candidate fixes: wheel the
   takeoff-area map points into the tracking prior during final approach (soft relocalization),
   or accept-and-fuse the landing rangefinder as a POSITION anchor (z) + loop the takeoff area
   visually at low altitude where N is richest (< 10 m AGL frames have the densest map).
