# 09 — Raspberry Pi / ARM64 Deployment

The fork builds and runs natively on ARM64 (validated on a **Raspberry Pi 5**, Debian 12, 4× Cortex-A76). This is the intended low-power headless target. Same code, same env knobs as x86.

## Install dependencies

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev libeigen3-dev \
                    libboost-serialization-dev libssl-dev \
                    libglew-dev libgl1-mesa-dev libegl1-mesa-dev
```

`libssl-dev` is easy to forget — the deterministic hash needs `openssl/md5.h`, and it is the usual first build failure.

## Build Pangolin from source

No apt package exists. If the device can't reach GitHub, clone on another machine and `rsync` the source over.

```bash
git clone --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
cmake -B build -DBUILD_EXTERN_GLEW=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
cmake --build build -j4 && sudo cmake --install build && sudo ldconfig
```

`-DBUILD_EXTERN_GLEW=OFF` forces the system GLEW (installed above) instead of an internet download. v0.6 installs a single `libpangolin.so` — that's fine, `find_package(Pangolin)` resolves it.

## Build the SLAM

```bash
./build.sh            # ~30–45 min on a Pi 5; use make -j2 if the board is thermally limited
```

`-march=native` is valid on aarch64 and `EIGEN_DONT_VECTORIZE` is already set — no x86 SIMD is required.

## Run (headless)

Identical to [doc 04](04-running-on-data.md); just don't set `ORB_VIEWER`. The compact deterministic recipe:

```bash
ORB_DETERMINISTIC=1 OMP_NUM_THREADS=1 ORB_CLAHE=3.0 ORB_TSJUMP_S=3.0 \
Examples/Monocular-Inertial/mono_inertial_euroc  Vocabulary/ORBvoc.txt  config.yaml  <dataset>  <dataset>/cam0_times.txt  pi_run
```

## Things to know on ARM

| Topic | Note |
|---|---|
| **Reproducibility** | md5s are bit-identical *within* an architecture, but **not** x86↔ARM (libm / FP contraction differ). Compare metrics across architectures, not hashes. |
| **Determinism = single-thread** | `ORB_DETERMINISTIC=1` runs single-threaded, so wall-time is ~2–3× the x86 box on an A76 core — it does **not** run real-time in this mode. Threaded mode is faster but non-reproducible. |
| **Onboard WiFi under load** | The Pi's WiFi can drop under sustained 100 %-CPU SLAM. For long unattended runs, prefer **Ethernet**, and launch runs **detached** (`setsid`/`nohup`) so an SSH drop can't kill them. |
| **Storage** | Datasets + build fit in a few GB; a fast SD or USB-SSD helps I/O-bound steps. |
| **Peak RAM** | Comparable to x86 (a few hundred MB up to a couple of GB depending on flight length and feature count). |

See [doc 10](10-troubleshooting-and-gotchas.md) for build/runtime gotchas.
