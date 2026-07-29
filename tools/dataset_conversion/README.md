# Dataset conversion → EuRoC layout

Scripts that convert **NORA** (high-resolution survey flights) and **ARDU**
(ArduPilot BIN-log historical flights) datasets into the **EuRoC directory
layout** consumed by ORB-SLAM3's `mono_euroc` / `mono_inertial_euroc`.

EuRoC output layout (produced by every converter here):

```
<out>/mav0/cam0/data/<ns>.png     mono grayscale frames (symlinked or resized)
<out>/mav0/imu0/data.csv          # ts[ns], gyro xyz [rad/s], accel xyz [m/s^2]
<out>/cam0_times.txt              one integer ns timestamp per kept frame
<out>/dataset_meta.json           window / epoch / offsets / source paths
<out>/ref_baro.csv  ref_rfnd.csv  ref_att.csv  ref_gps1.csv   (optional sidecars)
```

**Timestamp rebasing (all converters).** Every emitted timestamp is rebased to
an epoch (subtracted), because ORB-SLAM3's EuRoC loader parses each stamp into a
`double` and multiplies by `1e-9`; at absolute Unix-ns magnitude (~1.78e18) a
double's ULP is ~512 ns, which jitters/collapses IMU `dt` and breaks
preintegration. Rebased (~1e10–1e12) the ULP is sub-nanosecond. The epoch is
recorded in `dataset_meta.json` (add it back to recover Unix time); sidecar
references are rebased with the same epoch so trajectory, baro, mag and GPS all
share one clock.

## NORA family (high-res images + ArduPilot BIN RISI IMU)

| Script | Input format | Notes |
|---|---|---|
| `convert_nora_to_euroc.py` | NORA session: `high_res_images/<companion_ns>.png` (1640×1232 mono) + BIN RISI IMU | The primary converter (flights 538 / 542). Symlinks images, emits RISI-derived IMU, optional baro/rangefinder sidecars. |
| `convert_to_euroc.py` | golem27 "circle" recorder (800×600 mono, already calibrated) | First circle converter: symlinks images, trims takeoff/hard-landing, repairs interior gyro-zero rows by nearest-neighbour interpolation. |
| `convert_circle_to_euroc.py` | golem27 "circle" recorder | Parameterized rewrite of `convert_to_euroc.py` (`--start-ns/--end-ns/--epoch-ns`, IMU margin, no fixed landing trim). |
| `convert_datarecorder_to_euroc.py` | golem27 `data-recorder` (IMX219 1640×1232 + `ardupilot_imu_*.csv`, accel in g / gyro in deg/s) | Resizes to the calibrated 800×600, converts IMU units to SI, optional `--imu-shift-s` (kalibr) and `--imu-only`. |

## ARDU family (ArduPilot BIN historical fleet)

| Script | Purpose |
|---|---|
| `eval/hist_convert.py` | Convert one runnable historical flight to the common EuRoC eval layout: frame stamps from the hdf keypoints, per-flight clock bridge (`companion_ns = slope·fc_boot_ns + K`), BIN RISI → gyro/accel, kalibr time-shift, baro/rfnd/att/gps1 sidecars, and a per-flight `configs/hist/<key>.yaml` (KB4 intrinsics transformed 800×600 → crop → ×0.5 → 300×225). BIN parse cached in `eval_out/bincache/`. |
| `eval/hist_scan.py` | Flight scanner / selection helper (`FLIGHTS_ROOT`, BIN discovery, hdf chunk iteration, GPS-week→Unix). Imported by `hist_convert.py`. |

## Shared dependency

`nora_20260709/lib/nora_bin.py` — the ArduPilot **RISI** preintegrated-IMU parser
(`DA/DADT → gyro rad/s`, `DV/DVDT → accel m/s²`) and the companion-Unix-ns ↔
flight-controller `TimeUS` clock-bridge (`BridgeParams`, `risi_to_imu`). Both
`convert_nora_to_euroc.py` and `eval/hist_convert.py` import it; the directory
layout here (`nora_20260709/lib/` beside the NORA converters, `eval/` one level
below) preserves the original `sys.path` resolution so the imports work in place.

## Requirements

Python 3 with: `numpy`, `opencv-python` (`cv2`), `pyyaml`, `h5py`, `pymavlink`.

> **Note.** These are archived verbatim from the evaluation workspace. Input/output
> dataset paths inside the scripts are the original absolute workspace paths
> (e.g. `/home/kmro/praca/dev/...`) — adjust them (or the CLI args) for your
> environment before running.
