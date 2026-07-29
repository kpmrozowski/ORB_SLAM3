# exp1 — loop-closure audit (538 scale drift)

**Hypothesis:** survey re-visits terrain, so intra-map loops should fire and fix scale/horizontal
drift. If they don't help, either no loops fire, or they fire but get rejected.

**Method (read-only, no new run):** grepped all `logs/run_nora538_half_*.log` for
`Loop detected` / `BAD LOOP` / `Merge`; read the gate in `ORB_SLAM3/src/LoopClosing.cc`.

**Findings:**
- Loops ARE detected in several 538 runs (baro4: 13, plain clahe: 12, baro6u baseline: 1).
  Single map (baseline atlas = 1 map / 693 KFs), 0 merges → these are **intra-map loops**
  (exactly what corrects drift). Detection reaching "Loop detected" means 3 Sim3 coincidences
  passed geometric validation — the BoW/place-recognition pipeline works.
- **100% of detected loops are rejected as `BAD LOOP!!!`** at `LoopClosing.cc:240`:
  `if (fabs(phi(0))<0.008f && fabs(phi(1))<0.008f && fabs(phi(2))<0.349f)` else BAD.
  `phi` = loop relative-rotation vector (roll, pitch, yaw). Roll/pitch tol = **0.008 rad = 0.46°**.
- Observed rejected phi across logs: **yaw always small** (~0.08–0.16 rad, consistently ≈ −0.10,
  well under the 0.349 yaw tol) → signature of TRUE loops with small consistent yaw drift.
  Roll/pitch scatter 0.05–0.25 rad (gravity-alignment noise) with a few wild outliers (0.4–0.63).
  Baseline baro6u rejected loop: `phi=(-0.088,-0.149,-0.116)` → roll 5°, pitch 8.5° (>>0.46°).
- **Key insight:** when a loop IS accepted, lines 248–250 force roll/pitch to zero anyway
  (IMU gravity trusted) before applying — so the gate rejects loops whose roll/pitch it would
  discard regardless. And loop detection only runs post-BA2 (early return at LoopClosing.cc:341),
  so GetIniertialBA2() is always true → roll/pitch always zeroed on accept. Relaxing roll/pitch
  tol is therefore safe.

**Baseline metric (eval/combined_plot.py, gps_ref_538.csv):**
H-scale 1.011 · H-RMSE 292 m (3973 matched) · vATE 20.2 m (v-scale 0.71) · first pose t=33.2s.

**Verdict:** root cause of "no loop help" = over-tight inertial roll/pitch acceptance gate, not
missing candidates. Case is exp2 (relax gate), NOT exp3 (no candidates).

**Next:** make roll/pitch (and yaw) loop tol env-configurable (default 0.008/0.349 preserves
stock), run baro6u recipe with ORB_LOOP_RP_TOL≈0.20, measure H-RMSE/H-scale/vATE.
