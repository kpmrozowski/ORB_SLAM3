# 02 — Dataset conversion + post-takeoff trim

## Sources (READ-ONLY — never modify /media; converter only symlinks)
- 538 session: `/media/kmro/datasets/dops/vio/20260709/538_golem27_2026-07-09T17-02-39`
- 542 session: `/media/kmro/datasets/dops/vio/20260709/542_golem27_2026-07-09T17-26-03.temp`
  - `high_res_images/<companion_unix_ns>.png` — mono 1640×1232 grayscale
- BIN cache (preparsed RISI IMU / GPS / baro / rfnd / mag): `nora_20260709/cache/` (npz)

## Converter: `convert_nora_to_euroc.py`
Turns a session into EuRoC layout. Key facts (read its docstring):
- IMU = ArduPilot BIN `RISI` preintegrated @400 Hz (gyro=DA/DADT, accel=DV/DVDT), stamped on the
  companion↔TimeUS clock, cam-imu shift `-0.038675 s` (kalibr).
- **All emitted timestamps are REBASED by `--epoch-ns`** (subtract; add back for Unix). Rebasing is
  required for numerical safety (double ULP at 1.78e18 ns is ~512 ns → breaks IMU dt).
- Images are **symlinked** at full 1640×1232 (nothing copied) → the EuRoC dataset is full-res for free.
- Emits `mav0/cam0/data/<rebased_ns>.png`, `mav0/imu0/data.csv`, `cam0_times.txt`, `ref_baro.csv`,
  `ref_rfnd.csv`, `dataset_meta.json` (records epoch_ns, start/end).

## The "moving-start" trim (this session's key dataset step)
The flights begin STATIC on the ground → no IMU excitation → estimators can't observe scale/gravity at
init. The user supplied the "accel jumps +5 %" timestamps to start after motion begins. Rebuild with the
new `--start-ns` but the **ORIGINAL `--epoch-ns`** so the trimmed set stays on the SAME clock as
`gps_ref_*.csv` / NORA (only starts later):

```bash
# 538: start rebased t=3.87 s (skip ~3.9 s static)
python3 convert_nora_to_euroc.py --session <538 session> --cache nora_20260709/cache \
   --out dataset/nora538_move --start-ns 1783605978430903808 \
   --end-ns 1783606532881061376 --epoch-ns 1783605974565266432
# 542: start rebased t=26.19 s (skip ~26 s static)
python3 convert_nora_to_euroc.py --session <542 session> --cache nora_20260709/cache \
   --out dataset/nora542_move --start-ns 1783607231156786688 \
   --end-ns 1783607365295137536 --epoch-ns 1783607204961902080
```
Original epochs/ends come from each `dataset/nora5*_full/dataset_meta.json`.

**Validation of the trim** (`eval/... ` inline, see 06): accel-magnitude std jumps at the start points —
538: 0.53→3.14 m/s² (**×5.9**), 542: 0.39→3.15 (**×8.1**). Confirms genuine motion onset.

Result: `nora538_move` = 4197 frames / 554.5 s; `nora542_move` = 1267 frames / 134.1 s.

## 2026-07-15 afternoon additions: 4 golem27_home datasets (different source formats)
From `/home/kmro/praca/dev/datasets/droneops-calibrations/golem27_home/`, converted alongside 538/542:

| dataset | source format | frames/span | notes |
|---|---|---|---|
| `dataset/165608_move` | data-recorder, start 124648286663 | 3325 / 223.6 s | baro ✓ (Δ32.7 m); IMU 200 Hz |
| `dataset/163613_move` | data-recorder, start 209225154969 | 878 / 59.3 s | **source IMU/baro truncated** at ~60 s (camera runs 205 s); baro ✓ |
| `dataset/162710_move` | data-recorder, start 156539326898 | 1927 / 128.8 s | baro ✓ (Δ31.7 m); IMU ~394 Hz |
| `dataset/circle_move`  | circle fmt, start 1775118917575200256 (takeoff) → end 1775119066e9 (pre-landing) | 2935 / 148.4 s | RECONVERTED to the circling flight (first attempt at start 1775119067… was post-landing). 9 gyro-zeros repaired, 0 accel spikes. Viewer note: window opens on physical `:0`; run_dataset.sh now stdbuf-line-buffered |

- **data-recorder format** (`cam_0/<boot_ns>.png` 1640×1232, `ardupilot_imu_*.csv` accel-g/gyro-dps, `ardupilot_baro_*.csv` altitude_m): converter `convert_datarecorder_to_euroc.py` EXTENDED with `--native-symlink` (no resize, native symlinks, skips 0-byte) + automatic `ref_baro.csv` sidecar (`#timestamp [ns],alt_m`). Boot-ns (~1e11) needs NO rebase. IMU dropout gaps up to ~3.8 s kept raw (`--fill-imu-hz 200` if a run stalls).
- **circle format** (`cam0/<unix_ns>.png` natively 800×600, `imu0.csv` SI omega/alpha with gyro-dropout rows): NEW `convert_circle_to_euroc.py` (argparse: --src/--out/--start-ns/--end-ns/--epoch-ns; REBASES — unix-ns needs it; interior gyro-zero interpolation). Configs `configs/circle_{mono,mi}_800native.yaml` = native camchain scaled per-axis (fx,cx ×800/1640; fy,cy ×600/1232 → fx 278.2378 fy 278.2429 cx 384.8398 cy 310.2763; D unchanged), fps 21 (integer required!), IMU.Frequency 372.09. Pangolin: `ORB_VIEWER=1 ORB_TSJUMP_S=6.0 ./run_dataset.sh dataset/circle_move mono configs/circle_mono_800native.yaml circle_view`.

## ⚠ DATASET DEFECT: 56 zero-byte frames in the 542 session
`nora542_move` symlinks **56 zero-byte source PNGs** (e.g. `156667267584.png` →
`…/high_res_images/1783607361629169664.png`, size 0). ORB-SLAM3 aborts on the first one (~frame 1212)
with "Failed to load image"; VINS just publishes an empty image. 538 has **0** corrupt frames.

**Fix (do this before trusting any 542 result):** make the converter skip 0-byte source PNGs, then rebuild
`nora542_move`. Minimal patch to `convert_nora_to_euroc.py` `list_frame_stamps()` (it already lists PNGs):
```python
# in list_frame_stamps, replace the final return with one that drops empty source files:
return [stamp for stamp in stamps
        if start_ns <= stamp <= end_ns
        and os.path.getsize(os.path.join(image_dir, f"{stamp}.png")) > 0]
```
(datasets are read-only, so we skip the bad frames rather than repair the source; the continuous IMU
bridges the small gaps, same as the camera-drop handling.)
