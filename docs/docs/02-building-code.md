# 02 — Building the Code

Standard ORB-SLAM3 build (CMake + a helper script). Targets **C++14**, Linux, x86-64 or ARM64.

## Dependencies

| Dependency | Min version | Install (Debian/Ubuntu) |
|---|---|---|
| CMake, g++, make | — | `sudo apt install build-essential cmake` |
| OpenCV | 4.4 | `sudo apt install libopencv-dev` |
| Eigen | 3.1 | `sudo apt install libeigen3-dev` |
| Pangolin | 0.6+ | build from source (no apt package) — see below |
| Boost.Serialization | — | `sudo apt install libboost-serialization-dev` |
| OpenSSL (libcrypto) | — | `sudo apt install libssl-dev` |

`DBoW2`, `g2o`, and `Sophus` are **vendored** in `Thirdparty/` and built by the script — no separate install.

> **Pangolin** has no apt package. Build it once from source:
> ```bash
> git clone --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git
> cd Pangolin && cmake -B build -DBUILD_EXTERN_GLEW=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
>   -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF && cmake --build build -j && sudo cmake --install build
> ```
> Needs `libglew-dev libgl1-mesa-dev libegl1-mesa-dev`.

## Build

```bash
./build.sh          # builds Thirdparty, untars the vocabulary, then the main lib + examples
```

`build.sh` runs, in order: DBoW2 → g2o → Sophus → untar `Vocabulary/ORBvoc.txt` → main library + `Examples/*` (`make -j4`).

To rebuild only the main library after editing `src/`:

```bash
cmake --build build -j            # or: cd build && make -j4
```

## Verify

```bash
ls Examples/Monocular-Inertial/mono_inertial_euroc     # the binary
ls lib/libORB_SLAM3.so                                  # the library
```

## Common build errors

| Error | Fix |
|---|---|
| `openssl/md5.h: No such file` | `sudo apt install libssl-dev` (the deterministic hash uses OpenSSL) |
| `Could NOT find Pangolin` | Build & install Pangolin from source (above) |
| `libpango_*.so` vs GNOME `libpango-1.0` | Different libraries — you need **Pangolin**, not GNOME Pango |
| g2o/DBoW2 SIMD errors on ARM | `-march=native` is valid on aarch64; `-DEIGEN_DONT_VECTORIZE` is already set — no x86 intrinsics are used |

Building on a Raspberry Pi / ARM64? See [doc 09](09-raspberry-pi-arm-deployment.md).
