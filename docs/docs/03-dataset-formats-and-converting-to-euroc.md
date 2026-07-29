# 03 — Dataset Formats & Converting to EuRoC

ORB-SLAM3 reads the **EuRoC layout**. Raw flight data comes in several formats; converters in [`../../tools/dataset_conversion/`](../../tools/dataset_conversion/) turn each into EuRoC.

## Target: the EuRoC layout

```
<dataset>/
├── mav0/
│   ├── cam0/data/<ns>.png        mono grayscale frames
│   └── imu0/data.csv             # ts[ns], gyro xyz [rad/s], accel xyz [m/s^2]
├── cam0_times.txt                one integer ns timestamp per frame
├── dataset_meta.json            window / epoch / source paths
└── ref_baro.csv  ref_mag.csv  ref_rfnd.csv  ref_att.csv  ref_gps1.csv   (optional sidecars)
```

The `ref_*.csv` **sidecars** are ground-truth / fusion inputs (barometer altitude, magnetometer field, rangefinder range, attitude, authentic GPS). They are optional for running but required for fusion ([doc 04](04-running-on-data.md)) and scoring ([doc 05](05-evaluating-results.md)).

## Input families & converters

| Converter | Input | Notes |
|---|---|---|
| `convert_nora_to_euroc.py` | **NORA** session: `high_res_images/<ns>.png` (1640×1232) + ArduPilot BIN RISI IMU | Primary — flights 538 / 542. Symlinks images, emits baro/rfnd sidecars. |
| `convert_to_euroc.py` | golem27 **circle** recorder (800×600, calibrated) | Trims takeoff/landing, repairs interior gyro-zero rows. |
| `convert_circle_to_euroc.py` | golem27 **circle** recorder | Parameterized (`--start-ns/--end-ns/--epoch-ns`). |
| `convert_datarecorder_to_euroc.py` | golem27 **data-recorder** (1640×1232 + IMU csv in g / deg/s) | Resizes to 800×600, converts IMU to SI. |
| `eval/hist_convert.py` | **ARDU** (ArduPilot BIN historical fleet) | Per-flight clock bridge, RISI IMU, all sidecars, plus a generated `configs/hist/<key>.yaml`. |

All share `nora_20260709/lib/nora_bin.py` — the RISI IMU parser + companion-ns↔flight-controller clock bridge. Full details + requirements in the [converter README](../../tools/dataset_conversion/README.md).

## Timestamp rebasing (important)

Every converter **rebases timestamps** (subtracts an epoch, recorded in `dataset_meta.json`).

> **Why:** ORB-SLAM3 parses each stamp into a `double`. At absolute Unix-ns magnitude (~1.78e18) a double's step is ~512 ns, which jitters IMU `dt` and **breaks preintegration**. Rebased (~1e10), the step is sub-nanosecond. Add the epoch back to recover Unix time.

## Requirements

Python 3 + `numpy opencv-python pyyaml h5py pymavlink`.

## Example

```bash
# NORA flight → EuRoC (paths inside the script are workspace-absolute; adjust as needed)
python3 tools/dataset_conversion/convert_nora_to_euroc.py --help

# ArduPilot historical flight → EuRoC
python3 tools/dataset_conversion/eval/hist_convert.py --help
```

Gotchas (e.g. the 56 zero-byte PNGs in flight 542) are collected in [doc 10](10-troubleshooting-and-gotchas.md).
