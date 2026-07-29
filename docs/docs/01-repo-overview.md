# 01 — Repository Overview

A fork of **ORB-SLAM3** tuned for **monocular-inertial VIO on GPS-denied / GPS-spoofed aerial survey flights**. Stock ORB-SLAM3 loses metric scale and fails to initialize at cruise altitude; this fork adds altitude/heading fusion, robustness knobs, and a deterministic evaluation mode.

## What's different from upstream (one line)

> Barometer + magnetometer tight fusion, hardened loop closing, configurable init gates, a fully deterministic mode, and an early-divergence guard — **all env-gated, all default-off** (unset → stock ORB-SLAM3).

Full technical write-up: [`../DEVELOPMENT_REPORT.md`](../DEVELOPMENT_REPORT.md). Day-by-day research log: [`../diary/`](../diary/).

## Where things live

| Path | What |
|---|---|
| `src/`, `include/` | SLAM core (fork changes are here) |
| `include/BaroFusion.h`, `include/MagFusion.h` | Altitude / heading fusion edges + loaders |
| `include/FeaturePrefetcher.h`, `src/FeaturePrefetcher.cc` | Off-thread ORB pre-extraction (offline speed) |
| `include/DeterministicOrder.h` | `IdLess` ordering for reproducible runs |
| `Examples/Monocular-Inertial/mono_inertial_euroc.cc` | The runnable binary (main entry point) |
| `Vocabulary/ORBvoc.txt.tar.gz` | ORB vocabulary (untarred by `build.sh`) |
| `Thirdparty/{DBoW2,g2o,Sophus}` | Vendored deps, built in-tree |
| `tools/dataset_conversion/` | NORA & ArduPilot → EuRoC converters ([doc 03](03-dataset-formats-and-converting-to-euroc.md)) |
| `docs/` | This documentation, the report, and the research diary |

The **evaluation harness** (dataset configs, scoring scripts, run wrappers) lives one level up in the `orbslam3-eval/` workspace (`eval/`, `eval/ic_tune/`, `configs/hist/`), not inside this repo.

## Branch model

| Branch | Purpose |
|---|---|
| `master` | Pristine upstream ORB-SLAM3 |
| `dev` | Develop branch — the fusion / loop / init / determinism work (12 commits vs master) |
| `add-docs` | `dev` + this documentation + the dataset converters |

## Design principle

Every modification is behind an `ORB_*` environment variable and defaults to **exact upstream behavior**. This keeps the fork rebase-friendly and makes every experiment reversible — see the full knob list in [doc 08](08-environment-variable-reference.md).

## Read next

1. [Build the code](02-building-code.md)
2. [Prepare a dataset](03-dataset-formats-and-converting-to-euroc.md)
3. [Run on data](04-running-on-data.md)
4. [Evaluate results](05-evaluating-results.md)
5. [What works](06-vanilla-algo-all-good-mods.md) · [what didn't](07-tested-not-improving-mods.md)
