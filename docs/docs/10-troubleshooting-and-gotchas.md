# 10 — Troubleshooting & Gotchas

Quick lookup for the failures you're most likely to hit.

## Build

| Symptom | Cause | Fix |
|---|---|---|
| `openssl/md5.h: No such file` | `libssl-dev` missing | `sudo apt install libssl-dev` |
| `Could NOT find Pangolin` | Pangolin not installed (no apt package) | Build from source ([doc 02](02-building-code.md) / [09](09-raspberry-pi-arm-deployment.md)) |
| Pangolin auto-download fails (offline board) | `BUILD_EXTERN_GLEW=ON` tries to fetch GLEW | Configure Pangolin with `-DBUILD_EXTERN_GLEW=OFF` + system `libglew-dev` |
| Confusing `libpango-1.0` present but still "not found" | that's GNOME **Pango** (text), not **Pangolin** | Install Pangolin (`libpango_*.so` / `libpangolin.so`) |

## Data preparation

| Symptom | Cause | Fix |
|---|---|---|
| Run aborts partway (e.g. flight 542 at ~frame 1212) | zero-byte / corrupt source PNGs | Skip 0-byte PNGs in the frame lister (the converter does this) |
| IMU `dt` jitters / preintegration garbage | absolute Unix-ns timestamps lose precision as `double` | Use the converters' **timestamp rebasing** ([doc 03](03-dataset-formats-and-converting-to-euroc.md)); never feed raw Unix-ns |
| Map resets at every camera drop | multi-second frame gaps trigger the 1 s reset | Raise `ORB_TSJUMP_S` (e.g. 3.0) so the IMU preintegrates across the gap |
| `import nora_bin` fails when running a converter | moved script out of its layout | Keep `nora_20260709/lib/nora_bin.py` beside the converters ([converter README](../../tools/dataset_conversion/README.md)) |

## Runtime / results

| Symptom | Cause | Fix |
|---|---|---|
| Segfault at shutdown on a run that tracked nothing | empty-map trajectory save (fixed in this fork) | Ensure you're on `dev`/`add-docs`, not stock upstream |
| Position explodes, scale ≈ 0 in first minute | catastrophic init at altitude (weak excitation) | Add `ORB_BARO_FRAME_SIGMA=0.5`; enable the divergence guard to fail fast |
| Vertical fit lags the true climb | baro fusion lag ∝ σ | Lower `ORB_BARO_SIGMA` (0.075–0.15) — but not below ~0.05 (too stiff) |
| Tracking collapses on low-contrast footage | no contrast equalization | Set `ORB_CLAHE=3.0` (load-bearing on aerial data) |
| Small vertical ATE but map looks tilted | relative-datum baro edges are tilt-blind | Known limitation — a small vATE does **not** imply globally-correct z |
| Every run gives a different trajectory | thread nondeterminism | Use `ORB_DETERMINISTIC=1` for A/B experiments |
| Rangefinder fusion collapses scale over hills | rangefinder measures AGL, not a fixed datum | Use the barometer over relief; rangefinder only on flat, in-range flights |

## Efficiency

| Want | Do |
|---|---|
| Faster offline runs | `ORB_PREFETCH=all ORB_NO_PACE=1` (bit-identical, off-thread extraction) |
| Don't waste time on doomed runs | `ORB_DIVERGE_VMAX=30 ORB_DIVERGE_COUNT=4` (stop + save partial) |
| Run many flights at once | deterministic mode makes processes parallel-safe → `det_queue.sh <queue> <batch> <parallel>` |

## Reproducibility caveats

- md5s match **within** one machine/architecture, not across x86↔ARM — compare metrics cross-arch.
- One nondeterminism source remains (an uninitialized post-init heap read); ~1 in 10 runs of the hardest flight can still flip. See [doc 07](07-tested-not-improving-mods.md).
