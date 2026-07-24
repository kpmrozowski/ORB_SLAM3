# mem-budget worktree baselines (Task W0)

Recorded 2026-07-24 by Task W0. All P0-P4 md5 gates compare against the md5s in this file,
**running the worktree binary via `eval_fast/run_flight.sh`** (see "md5-gate scope" below).

> **P0.5 re-baseline in effect:** the canonical md5s/coverage/RSS for P1+ are in the
> "P0.5 re-baseline" section at the end of this file. The W0 tables below are kept as
> pre-P0.5 history (they were recorded with a binary whose trajectory depended on
> uninitialized-heap luck — see `.superpowers/sdd/task-P0.5-report.md`).

## Provenance

| item | value |
|---|---|
| worktree | `.worktrees/mem-budget`, branch `mem-budget` |
| HEAD at baseline | `fd5e479` ("chore: ignore .worktrees") = source-identical to `8591f9a` (fd5e479 touches only `.gitignore`) |
| planned base in the W0 brief | `c6f6e62` — superseded: `8591f9a` (viewer-only GPS[1] overlay) landed on `ic-integration` before W0 ran, so the worktree is based on current HEAD `8591f9a`+gitignore |
| binary | `Examples/Monocular-Inertial/mono_inertial_euroc` + `lib/libORB_SLAM3.so`, built in-worktree (`build.sh`, then re-cmaked with the main checkout's cache flags: `CMAKE_CXX_FLAGS="-isystem /home/kmro/praca/dev/dmvio-loop/third_party/local/include"`, `CMAKE_{EXE,SHARED}_LINKER_FLAGS="-L<local>/lib -Wl,-rpath,<local>/lib"`), gcc 13.3.0, `-O3 -march=native` |
| vocabulary | worktree `Vocabulary/ORBvoc.txt`, md5 `5420bad0713bc97034dd2a9b2f0cc387` (bit-identical to main checkout's) |
| runner | `eval_fast/run_flight.sh <flight> <nf> <out_dir> [times_file]` — deterministic campaign env, real-PID RSS/CPU sampler (5 s interval), 4000 s in-sampler cap |
| deterministic env | `ORB_DETERMINISTIC=1 OMP_NUM_THREADS=1 ORB_CLAHE=3.0 ORB_TSJUMP_S=3.0 ORB_BARO_CSV=<ds>/ref_baro.csv ORB_BARO_SIGMA=0.15 ORB_BARO_GATE=6 ORB_BARO_FRAME_SIGMA=0.5 ORB_MAG_CSV=<ds>/ref_mag.csv ORB_MAG_SIGMA_DEG=10 ORB_DIVERGE_VMAX=30 ORB_DIVERGE_COUNT=4` (no viewer, no GPS overlay) |
| configs | `eval_fast/config_212_golem27.yaml` md5 `badf3505d6685d9c41677299a845540a` (byte-exact copy of the config the pre-W0 212 reference baseline ran with, NF=2500 already set); `eval_fast/config_182_golem27.yaml` md5 `f85264d29911438efc509b060a79ef55` (from `configs/hist/182_golem27_2026-02-27T15-07-52_noup.yaml`; 182 is absent from `flights.json`); NF applied per-run via `sed ORBextractor.nFeatures` into `<out_dir>/config.yaml`, sed verified by grep (run_viewer.sh:73-76 pattern) |

## Full-flight baselines (worktree binary @ 8591f9a source)

Runs of 2026-07-24 16:45-17:22 CEST, three in parallel, `nice -n 10`, alongside an unrelated
~14-process eval campaign (machine load ~19/20 cores; determinism is unaffected, wall time and
avg CPU are contention-inflated).

| flight | NF | frames | f_ md5 | kf_ md5 | peak VmHWM | avg CPU | median CPU (5s samples) | wall | coverage (RUN SUMMARY) |
|---|---|---|---|---|---|---|---|---|---|
| 212_golem27 | 2500 | 4138 | `ea91b2513f7bba2170fd5372acf362cf` | `f7dfc7682df02fa5317a9fabdb7843b5` | **2494 MB** | 79.9% | 80.0% (max 100.2) | 2025 s | `coverage 4110/4138 poses (99.32%) [COMPLETE]` |
| 182_golem27 | 2500 | 3074 | `c7bfe63d985e6a735add54d1a23d6138` | `729be766ea6d19f5410850ab4faabf24` | **928 MB** | 80.9% | 85.2% (max 100.4) | 361 s | `coverage 724/3074 poses (23.55%) [DIVERGED-ABORT]` |
| 182_golem27 | 5000 | 3074 | `6cd339f68b4ae26272f614299629c454` | `cb598e56c31c5ef0ffb9bb5a92cd3845` | **3231 MB** | 88.2% | 89.2% (max 100.6) | 2201 s | `coverage 3029/3074 poses (98.54%) [COMPLETE]` |

Notes:
* **182@NF2500 diverges-aborts** at frame 747 (`[DIVERGE] 4 consecutive KFs with |v| > 30 m/s (last 31.28 m/s, KF 377)`), so its
  peak-RSS number covers only ~24% of the flight — it is the *deterministic baseline behaviour* of this env/config (guard default
  action = abort), reproduced bit-identically by the md5 above, but NOT a full-flight memory ceiling. 182@NF5000 runs the full
  flight (more features -> tracking survives), so the NF=5000 row is the meaningful full-flight 182 memory baseline.
* Peak VmHWM read from `/proc/<binary pid>/status` by the sampler (real PID — the pre-W0 scratchpad runner's `timeout`-wrapper
  PID bug is fixed in `run_flight.sh`; the old runner's monitor CSV shows 2 MB peaks, its true peak came from a separate
  `final_stats.txt` read).
* avg CPU = total utime+stime jiffies / wall (whole-run); median CPU = per-5s-sample median (the Global-Constraints gate stat).

## Fast md5 gate (every implementer's cheap check)

`eval_fast/times_212_3min.txt` = first 800 lines of `dataset/hist_212_golem27/cam0_times.txt`
(168.9 s of data, ~4 min wall), md5 `8e07f9a89b8a61ff2dc3375c3f5159d4`.

```
bash eval_fast/run_flight.sh 212_golem27 2500 eval_runs/<name> eval_fast/times_212_3min.txt
```

| quantity | value |
|---|---|
| f_ md5 | `2446988e0a437b572f03e7d4d71b7f05` |
| kf_ md5 | `4f22a9572b3eab1193a56c9dafadbb5c` |
| coverage | `772/800 poses (96.50%) [COMPLETE]` |
| peak VmHWM | ~920-923 MB |
| reproducibility | verified: two independent runs bit-identical; also invariant under a lib-swap and under the cmake-flag alignment (5 runs total, see below) |

## Pre-existing reference baselines (NOT reproduced by this worktree — see verdict)

| flight | mode | NF | binary | f_ md5 | kf_ md5 | peak VmHWM | avg CPU | coverage |
|---|---|---|---|---|---|---|---|---|
| 212_golem27 | headless det | 2500 | main checkout @ `c6f6e62` (recorded 2026-07-24 ~15:42, pre-8591f9a-rebuild) | `090e98a2e08f83faf543e260336638a1` | `295ac75b7bf42d769effed0b18c5db2e` | 2485 MB | 63.9% | 4110/4138 (99.32%) COMPLETE |
| 243_golem17 | viewer | 2500 | (from plan Global Constraints; not re-measured in W0) | — | — | 3091 MB | — | — |

## Clean-baseline verdict (212@NF2500 md5 vs recorded c6f6e62 baseline): **MISMATCH — explained, not a code regression**

The worktree run produced `ea91b251...` vs the recorded `090e98a2...` (identical coverage 99.32%,
peak RSS 2494 vs 2485 MB). Root-cause isolation on the 3-min fast gate:

| run | exe | libORB_SLAM3.so | f_ md5 |
|---|---|---|---|
| A | worktree | worktree | `2446988e...` |
| B (repeat of A) | worktree | worktree | `2446988e...` (bit-identical -> determinism PROVEN) |
| A2 (after aligning worktree cmake flags to main's) | worktree | worktree | `2446988e...` (flags irrelevant) |
| C | main checkout | main checkout | `242348c6...` (differs) |
| D | worktree | **main's** (forced via LD_LIBRARY_PATH) | `2446988e...` (follows the exe, not the lib) |

Additional facts: source is identical (`8591f9a` diff is viewer-only: `GpsOverlay`/`MapDrawer::DrawGPS`/
`Viewer::Run`, none reachable headless); gcc identical (13.3.0); `Thirdparty/{g2o,DBoW2}` `.so` files are
**bit-identical** across the trees; g2o `config.h` identical; the two `mono_inertial_euroc` executables'
`.text` sections are **bit-identical**; the two `libORB_SLAM3.so` `.text` sections are same-size but
byte-different; configs byte-identical; ASLR is on (`randomize_va_space=2`) yet within-tree runs are
bit-reproducible.

Conclusion: deterministic-per-tree, different-across-trees behaviour is a **build/runtime-environment
reproducibility trait of this stack** (present even at one and the same commit between the main checkout
and the worktree), not an effect of `8591f9a` and not a bug introduced by W0. The recorded `090e98a2`
c6f6e62 number cannot be re-verified either way because the main binary was rebuilt (to 8591f9a) at 15:52
before W0 started.

### md5-gate scope going forward

* **P0-P4 bit-identical md5 gates are worktree-internal**: baseline md5s in THIS file vs a candidate
  built and run in THIS worktree via `run_flight.sh` (same exe path, default `BIN`). That comparison is
  proven bit-stable.
* **Cross-tree comparisons (worktree vs main checkout / campaign binaries) must use J-score / trajectory
  metrics, never md5.** This includes the final-integration gate.

## P0.5 re-baseline (2026-07-24, `p0.5-det-alloc-invariance`) — CANONICAL for P1+

P0.5 made the deterministic trajectory **allocation-invariant** (see
`.superpowers/sdd/task-P0.5-report.md`): two classes of never-written-member reads were fixed
(`Tracking::mnFramesToResetIMU` on the Settings config path, and the `MapPoint` track-scratch
family — `mTrackDepth` garbage used to decide the inertial optimizers' close-point chi2
branch). This is a one-time, expected md5 shift; the W0 rows above are pre-P0.5 history.

Acceptance matrix (all bit-identical, `eval_runs/p05v2_*`): {none ×2, `ORB_DET_CANARY`=64,
=1048576, =1048576,free} ∪ {`MALLOC_PERTURB_`=1, =2} ∪ {`ORB_MEM_STATS_CSV`=on × {no canary,
canary 1MB}} — nine runs, one distinct f_ md5, one distinct kf_ md5. `MALLOC_PERTURB_=1` is
the recommended cheap defensive companion gate for P1+ (flips every never-written malloc
byte; would have caught both P0.5 defects).

### Fast md5 gate (canonical for P1+)

| quantity | value |
|---|---|
| f_ md5 | `6d588726ce556625c924b1a43a72a5c8` |
| kf_ md5 | `eda1f7a384a4735906817d7d14a28371` |
| coverage | `772/800 poses (96.50%) [COMPLETE]` (unchanged vs W0) |
| peak VmHWM | ~915-925 MB (unchanged range) |
| reproducibility | nine-run acceptance matrix bit-identical (incl. none ×2) |

### Full-flight baselines (canonical for P1+; runs `eval_runs/p05v2_base_*`)

| flight | NF | f_ md5 | kf_ md5 | peak VmHWM | median CPU (5s) | wall | coverage (RUN SUMMARY) |
|---|---|---|---|---|---|---|---|
| 212_golem27 | 2500 | `875680f64df9813ea7bddc0a8a10157b` | `2f2b9b7a2bb1cf8f238bafd911551bab` | **2442 MB** | 65.3% (max 95.0) | 1363 s | `coverage 4110/4138 poses (99.32%) [COMPLETE]` (identical coverage to W0) |
| 182_golem27 | 2500 | `80ab36499829d29deeefbe2b827e3179` | `c08508fb187f6b666ae6fc8137cd094a` | **998 MB** | 66.6% (max 100.2) | 281 s | `coverage 887/3074 poses (28.85%) [DIVERGED-ABORT]` (abort at frame 910 / KF 470; W0: frame 747 / 23.55% — the old abort point was partly uninitialized-heap luck) |
| 182_golem27 | 5000 | `97690758aff4146fa2ed355540150192` | `b9f92562d136dce1ba2dabbf232a7bcd` | **3256 MB** | 78.2% (max 100.2) | 1393 s | `coverage 3029/3074 poses (98.54%) [COMPLETE]` (identical coverage to W0) |

Notes:
* Runs of 2026-07-24 ~20:05-20:55 CEST, three in parallel, `nice -n 10`, light machine load
  (wall/CPU not directly comparable to the contention-inflated W0 numbers; determinism
  unaffected).
* 212@2500 and 182@5000 reproduce their W0 coverage EXACTLY (same pose counts) — the fixes
  do not change healthy-run tracking outcomes on the gate flights. 182@2500 remains
  DIVERGED-ABORT (its baseline behaviour) but aborts later (frame 747 → 910, 23.55% →
  28.85%): the old abort point depended on uninitialized-heap garbage, which P0.5 removed;
  flagged in the P0.5 report per the >1%-coverage rule.
* Peak VmHWM: 212@2500 2442MB (W0 2494), 182@2500 998MB (W0 928; runs 163 frames longer),
  182@5000 3256MB (W0 3231, +0.8%).
