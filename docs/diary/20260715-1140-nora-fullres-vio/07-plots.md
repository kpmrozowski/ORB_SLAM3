# 07 — Trajectory PNG plots

## Generated this session (in `plots/`)
- `combined_538_native1640.png` — the ACTUAL native-1640 538 figure: ORB-MI 646-pose fragment
  (**H-scale 0.045 = 22× scale-collapse**, vATE 174 m), VINS legacy (H-scale 0.006, vATE 33.8 km) and
  VINS measured (0.016, 7.3 km) — the quantitative picture of the net-negative verdict. TUM conversions
  saved at `runs/nora538_move/vins_native1640{,_measimu}_tum.txt`.
- `combined_538_reference.png` — the 3-panel combined figure for the known-good **800-res** 538 MI
  trajectory (native-1640 MI saved no trajectory, so this is the reference). Panels: (1) horizontal
  Umeyama-2D vs NORA (H-scale 1.022, RMSE 308 m), (2) altitude vs baro on the shared clock with init-latency
  markers (vATE 58.6 m, v-scale 0.31), (3) matches/frame — overlaid with the native-1640 legacy + measured
  `frame_stats` so you can SEE the measured-noise match collapse vs legacy.
- `matches_542_example.png` — example `match_viz` per-pair curve (strided smoke; rerun stride 1 for final).

## Tools
### `eval/combined_plot.py` — the main per-flight figure (NORA vs ORB vs VINS, shared clock)
```bash
python3 eval/combined_plot.py --gps nora_20260709/gps_ref_538.csv --baro dataset/nora538_move/ref_baro.csv \
  --title "538 full-res moving-start" \
  --traj "ORB-MI=runs/nora538_move/mono_inertial/native1640/f_native1640.txt=tum" \
  --traj "VINS=runs/nora538_move/vins_native1640_tum.txt=tum" \
  --stats "ORB-MI=runs/nora538_move/mono_inertial/native1640/frame_stats.csv" \
  --out plots/combined_538.png
```
- Horizontal = Umeyama-2D (honest horizontal scale, not vertical-conflated). Vertical = `baro_ate.vertical_ate`.
- Every trajectory drawn at its TRUE timestamp; dotted vertical line = each estimator's init latency
  (this is the correct way to show the "timeshift" — as init latency, not a fake offset).
- `--diverge-t <s>` draws a magenta line where a run's height starts diverging.
- Add more `--traj name=path=tum` / `--stats name=csv` to overlay VINS, VINS-mag, measured variants, etc.
  (convert VINS vio.csv → TUM first with `eval/vins_to_tum.py`).

### `eval/baro_ate.py` — vertical ATE vs baro, per trajectory (overlay plot + metric/shape ATE)
### `eval/validate_nora.py` — 2-D GPS-anchored XY + vertical panel + PLY export (3 scale estimates)
### `eval/anchor_nora.py` — 2-D horizontal GPS anchor + baro vertical → metric nav PLY

## Deliverable convention (memory `feedback_no_python_no_html`)
Present results as **PLY (MeshLab) + PNG (GIMP) paths + numbers in text**. No HTML dashboards. `validate_nora.py`
/ `anchor_nora.py` already emit PLY point clouds for MeshLab. Metrics go in text/CSV.

## Pending (blocked on runs that save a trajectory)
The combined/baro_ate/anchor PNGs for the native-1640 runs cannot be made until those runs actually save a
`f_*.txt` (all MI runs ended with empty maps this session — see 00/04). Once fixed (RESUME), the commands
above produce them directly.
