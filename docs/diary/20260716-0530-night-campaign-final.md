# Night campaign 2026-07-16 final — 542 goal met; σ-lag law; 538 bonus transfer

**Window:** 03:08–~06:00, autonomous. **Goal:** any VIO on 542 not worse than NORA altitude.
**Verdict: vATE won, drift RMS par.** 51+ runs, 7 batches. Full data:
`runs/nora542_half/night542_summary.csv`, `runs/night538_transfer_summary.csv`, RESULTS.md §12.

## Headline numbers (542, GT = real baro, same airborne span t=27.6–156.6 s)

| | vATE | drift RMS | coverage |
|---|---|---|---|
| NORA (onboard EKF) | 1.42 | 1.44 | 100% |
| **MI + baro σ0.075** (median, n=14) | **1.15** | 1.53 | full airborne 9/14 draws |
| best draws | 0.95–0.97 | 0.91–1.31 | full |
| MI + baro σ0.15 (median, n=5) | 1.28 | 1.88 | **full airborne 5/5** |

User-facing figure: `plots/20260716/altitude_542_final.png` (3-panel: altitude overlay,
cumulative error, per-window drift bars — best σ0.075 draw + typical σ0.15 draw vs NORA).

## What the night established (chronological)

1. **Coverage myth**: 542 takeoff t=27.7 s ⇒ 77% pose coverage = 100% of the airborne flight.
2. **nf1500 lottery**: yesterday's 1.49 "champion" = no-baro lucky draw (re-draws 1.8–9.9);
   tight baro on nf1500 = init death (3/5). nf5000 is mandatory under tight σ.
3. **Fusion-lag law** (the night's key physics): EdgeBaroZ makes z reproduce the barometer
   late, lag ∝ σ (σ1.0→~1.4 s, σ0.3→0.5 s, no-baro→0). Measured by shift-scanning GT against
   each trajectory. On 542's 2.5 m/s verticals, this lag WAS most of the fused vATE.
4. **σ ladder**: 2.55 (σ0.3) → 1.28 (σ0.15) → **1.15 (σ0.075)** → 1.70 (σ0.05, floor).
5. **Frame-rate baro edge** (`ORB_BARO_FRAME_SIGMA`, new C++, flag-gated default-off):
   neutral on RMS — the residual is draw noise + the baro's own landing-window ground-effect
   error (−2.3…−3.6 m in EVERY estimator incl. NORA). Kept: slight coverage benefit at σf0.5.
6. **Rejected**: ScaleRefinement+z-edges (v-scale thrash), σ0.05, frame σf≤0.25, rfnd-source
   at σ0.3 (no better than tightening raw baro).
7. **538 bonus transfer (RESULTS §13)**: same-span NORA bar = 0.98 vATE / 1.23 RMS. Tight σ
   healthy draws: σ0.075 → 1.81/6.5/7.2/8.0/12.0 (v-scale repaired 0.71→≈1.0, 67–98% cov);
   σ0.15 → 2.45/4.64 at 98%. Best draws match NORA's drift RMS (1.83 vs 1.72 full-span) and
   are within 2× of its vATE — vs 20× yesterday. Price: **~30% catastrophic-init rate**
   (v-scale ≤0.005 explosion or FIBA segfault in reset thrash) at ANY tight σ; not a σ0.15
   cliff (round-1 n=2 was the lottery). **Rounds 4+6: the frame-rate baro edge SUPPRESSES the
   fragility — σf0.5 went 9/10 healthy** (median ~7.2, worst healthy 20.19, v-scale ≈1.0, 82–98% cov;
   the per-frame z pin carries tracking through the pre-VIBA2 phase). Recommended 538 recipe:
   σ0.075 + gate6 + `ORB_BARO_FRAME_SIGMA=0.5`. Figure:
   `plots/20260716/altitude_538_transfer.png`.

## Infrastructure added

- `eval/night542_queue.sh` / `night542_queue2.sh` / `night_queue3.sh` — 2-slot queue runners
  (g2o-race safe) with per-run auto-eval (vATE/shape/v-scale/drift RMS/coverage) into summary
  CSVs; batch specs `eval/night542_batch{1..6}.txt`, `eval/night538_transfer{,2}.txt`.
- `Optimizer.cc::AddFrameBaroEdge` + env `ORB_BARO_FRAME_SIGMA` (see 20260716-0445 diary).
- Diaries: 20260716-0400 (batch 1 + lag discovery), -0515 (σ ladder), -0445 (frame edge), this.

## Open threads for the day shift

- 538 round-2 result (t538b) — if σ0.075 reproduces ~6–12 vATE at n=4, update RESULTS §10's 538
  row (20.2 → ~9) and make σ0.075 the default 538 recipe too.
- The remaining 542 gap to NORA's RMS (1.53 vs 1.44) is not vertical-fusion-fixable: it is
  landing-window baro error + map-draw noise. Next levers are horizontal: H-scale lottery
  (0.45–1.1 draws), loop-closure inertial gate (`ORB_LOOP_RP_TOL`, exp-2 thread from yesterday).
- Init latency: no VIO pose before takeoff+0 s is possible (static frames); NORA's EKF covers
  pre-takeoff trivially (zero motion). Claims should always be same-span.

## Post-06:00 addenda

- 542 unified recipe (σ0.075+f0.5) n=15: vATE median 1.27 (< NORA 1.42), 13/15 full coverage —
  reliability pick; plain σ0.075 stays the accuracy pick (1.15).
- Loop-gate relaxation composition on 538: verdict RETRACTED next morning — those 4 runs had
  ZERO loop detections (tilted draw was recipe draw-noise, not loops). Upstream is already 4-DoF
  on accepted inertial loops; see 20260716 morning 4-DoF hardening + rerun.
- **162710_move home flight: vATE 0.44–0.48 m / RMS 0.37–0.46 m @ ~98% coverage** (3/4 draws) with
  the unified recipe — 30× better than yesterday's 15.7/4.5.
- Home round 2: 163613 still unusable (59-s truncated log: segfault / 56-pose dud). 165608 with
  unified recipe: best draw vATE 0.64 / RMS 0.58 @63% cov but v-scale 0.67 (its known IMU-rate /
  calib question, not a fusion issue); second draw 8.2 @32%. 162710 remains the clean home win.
