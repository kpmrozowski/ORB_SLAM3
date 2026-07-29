#!/usr/bin/env python3
"""Reusable access to the cached ArduPilot BIN (00000026.BIN) RISI IMU stream and
the verified companion-clock <-> FC-boot (TimeUS) time bridge for the 20260709 flight.

Background
----------
RISI is ArduPilot replay preintegrated-IMU data. It carries NO TimeUS of its own; it
is stamped by the preceding RFRH (Replay Frame Header) TimeUS (FC boot microseconds).
Semantics: gyro[rad/s] = [DAX,DAY,DAZ]/DADT ; accel[m/s^2] = [DVX,DVY,DVZ]/DVDT.

Time bridge (see scripts/fit_bridges.py)
----------------------------------------
Three clocks are involved:
  * companion : the image-filename nanoseconds (== hdf5 'timestamp').
  * boot      : FC boot micros == RISI RFRH TimeUS == hdf5 time_since_boot / remote_timestamp.
  * gps_unix  : GPS-derived UTC (jittery, and partly spoofed in this dataset).

The RECOMMENDED, spoof-independent recipe maps the image clock directly to boot/TimeUS:

    companion_seconds = SLOPE * (TimeUS * 1e-6) + K            (fit resid 7.2 ms)

fitted from hdf5 (timestamp, time_since_boot) pairs. It was VERIFIED against an
independent BIN-GPS bridge (flight-time GPS cluster) to <= 12 ms across the image window.
An 81 s offset seen in the raw hdf5 'time_unix_usec' is a spoofing/EKF-time artifact
that does NOT affect this companion<->boot recipe.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np

GRAVITY = 9.80665
DEFAULT_CACHE_DIR = "/home/kmro/praca/dev/orbslam3-eval/nora_20260709/cache"
DEFAULT_TAG = "bin26"
COVERING_BIN = "/media/kmro/datasets/dops/vio/20260709/BIN/00000026.BIN"


@dataclass(frozen=True)
class BridgeParams:
    """Fitted constants for the companion<->TimeUS map (Bridge B, recommended)."""

    slope: float = 0.999966142          # companion seconds per boot second (-33.86 ppm)
    k_seconds: float = 1783604781.851651  # companion Unix seconds at TimeUS = 0
    # Independent BIN-GPS cross-check constants (Bridge A, flight cluster) -- documentation only.
    gps_a: float = 9.999731429515e-07     # gps_unix per TimeUS microsecond
    gps_b: float = 1783604781.739940      # gps_unix at TimeUS = 0
    gps_instance: int = 1                 # authentic GPS instance (GWk 2426, July 2026)

    @property
    def k_nanoseconds(self) -> int:
        """companion nanoseconds at TimeUS = 0 as an exact int64 offset."""
        return int(round(self.k_seconds * 1e9))


DEFAULT_PARAMS = BridgeParams()


# --------------------------------------------------------------------------- loading
def load_cached(cache_dir: str = DEFAULT_CACHE_DIR, tag: str = DEFAULT_TAG) -> dict:
    """Load the per-message-type .npz caches into a dict of dicts of numpy arrays.

    Returns keys: 'risi', 'gps', 'baro', 'rfnd', 'mag'. Each value is a dict whose
    keys are the stored column names (see scripts/parse_bin.py).
    """
    message_types = ("risi", "gps", "baro", "rfnd", "mag")
    loaded = {}
    for message_type in message_types:
        npz_path = os.path.join(cache_dir, f"{tag}_{message_type}.npz")
        with np.load(npz_path) as handle:
            loaded[message_type] = {name: handle[name] for name in handle.files}
    return loaded


# --------------------------------------------------------------------------- IMU
def risi_to_imu(risi_arrays: dict):
    """Convert cached RISI (instance 0) arrays to calibrated-free IMU samples.

    Returns (timeus[int64], gyro_xyz[rad/s, Nx3], accel_xyz[m/s^2, Nx3]).
    Samples with a non-positive integration time are dropped.
    """
    timeus = risi_arrays["timeus"].astype(np.int64)
    delta_angle_dt = risi_arrays["dadt"]
    delta_velocity_dt = risi_arrays["dvdt"]

    valid = (delta_angle_dt > 0.0) & (delta_velocity_dt > 0.0)
    timeus = timeus[valid]
    delta_angle = np.column_stack(
        (risi_arrays["dax"][valid], risi_arrays["day"][valid], risi_arrays["daz"][valid])
    )
    delta_velocity = np.column_stack(
        (risi_arrays["dvx"][valid], risi_arrays["dvy"][valid], risi_arrays["dvz"][valid])
    )
    gyro_xyz = delta_angle / delta_angle_dt[valid, None]
    accel_xyz = delta_velocity / delta_velocity_dt[valid, None]
    return timeus, gyro_xyz, accel_xyz


# --------------------------------------------------------------------------- time bridge
def timeus_to_companion_ns(timeus, params: BridgeParams = DEFAULT_PARAMS) -> np.ndarray:
    """Map FC-boot TimeUS (microseconds) to companion/image-clock nanoseconds (int64).

    companion_ns = round(SLOPE * 1000 * TimeUS) + K_ns. The large K offset is kept as
    an integer so nanosecond values (~1.78e18) do not lose precision in float64.
    """
    timeus_array = np.asarray(timeus, dtype=np.float64)
    scaled = np.round(params.slope * 1000.0 * timeus_array).astype(np.int64)
    return scaled + np.int64(params.k_nanoseconds)


def companion_ns_to_timeus(stamp_ns, params: BridgeParams = DEFAULT_PARAMS) -> np.ndarray:
    """Inverse map: companion/image-clock nanoseconds -> FC-boot TimeUS (microseconds, int64)."""
    stamp_array = np.asarray(stamp_ns, dtype=np.int64)
    delta_ns = (stamp_array - np.int64(params.k_nanoseconds)).astype(np.float64)
    return np.round(delta_ns / (params.slope * 1000.0)).astype(np.int64)


# --------------------------------------------------------------------------- convenience series
def baro_alt(baro_arrays: dict, params: BridgeParams = DEFAULT_PARAMS, instance: int = 0):
    """Barometric altitude series on the companion clock.

    Returns (companion_ns[int64], alt_m[float64]) for the requested baro instance.
    """
    keep = baro_arrays["inst"] == instance
    companion_ns = timeus_to_companion_ns(baro_arrays["timeus"][keep], params)
    return companion_ns, baro_arrays["alt"][keep]


def rfnd_dist(rfnd_arrays: dict, params: BridgeParams = DEFAULT_PARAMS, valid_only: bool = True):
    """Down-facing rangefinder distance series on the companion clock.

    valid_only keeps only Stat == 4 (RangeFinder::Status::Good) samples.
    Returns (companion_ns[int64], dist_m[float64]).
    """
    keep = np.ones(rfnd_arrays["dist"].shape, dtype=bool)
    if valid_only:
        keep = rfnd_arrays["stat"] == 4
    companion_ns = timeus_to_companion_ns(rfnd_arrays["timeus"][keep], params)
    return companion_ns, rfnd_arrays["dist"][keep]


# --------------------------------------------------------------------------- self-test
def _self_test() -> None:
    data = load_cached()
    timeus, gyro_xyz, accel_xyz = risi_to_imu(data["risi"])
    accel_mag = np.linalg.norm(accel_xyz, axis=1)
    gyro_mag = np.linalg.norm(gyro_xyz, axis=1)
    seconds = (timeus - timeus[0]) * 1e-6

    print("== nora_bin self-test (00000026.BIN) ==")
    print(f"RISI samples          : {timeus.size}  (~{1e6/np.median(np.diff(np.unique(timeus))):.1f} Hz)")
    print(f"TimeUS span           : {timeus.min()} .. {timeus.max()}")

    # rest window: quietest 10 s in the first 120 s (pre-takeoff)
    best_start, best_score = 1.0, np.inf
    for start in np.arange(1.0, 110.0, 2.0):
        window = (seconds >= start) & (seconds < start + 10.0)
        if window.sum() < 100:
            continue
        score = gyro_mag[window].var() + accel_mag[window].var()
        if score < best_score:
            best_score, best_start = score, start
    rest = (seconds >= best_start) & (seconds < best_start + 10.0)
    print(f"rest window           : t=[{best_start:.0f},{best_start+10:.0f}] s  n={rest.sum()}")
    print(f"(a) accel |a| at rest : {accel_mag[rest].mean():.5f} +/- {accel_mag[rest].std():.5f} m/s^2 (expect {GRAVITY})")
    print(f"(b) gyro bias per axis: {gyro_xyz[rest].mean(axis=0)} rad/s")

    companion_ns, alt = baro_alt(data["baro"])
    print(f"(c) BARO alt          : min={alt.min():.2f} max={alt.max():.2f} m  (companion-stamped, n={alt.size})")

    rfnd_ns, dist = rfnd_dist(data["rfnd"], valid_only=True)
    total_rfnd = data["rfnd"]["dist"].size
    print(f"(d) RFND valid        : {dist.size}/{total_rfnd} ({100*dist.size/total_rfnd:.1f}%)  "
          f"median={np.median(dist):.2f} m  max={dist.max():.2f} m")

    field = np.column_stack((data["mag"]["magx"], data["mag"]["magy"], data["mag"]["magz"]))
    field = field[data["mag"]["inst"] == 0]
    print(f"(e) MAG |B| median    : {np.median(np.linalg.norm(field, axis=1))*0.1:.2f} uT")

    # time-bridge round trip + a concrete image stamp
    sample_stamp = 1783606253723163904
    round_trip = timeus_to_companion_ns(companion_ns_to_timeus(sample_stamp))
    print(f"\ntime bridge (Bridge B): SLOPE={DEFAULT_PARAMS.slope:.9f}  K={DEFAULT_PARAMS.k_seconds:.6f} s")
    print(f"  image {sample_stamp} -> TimeUS {int(companion_ns_to_timeus(sample_stamp))}"
          f" -> companion_ns {int(round_trip)}  (round-trip err {int(round_trip)-sample_stamp} ns)")
    first_ns = int(timeus_to_companion_ns(timeus[0]))
    last_ns = int(timeus_to_companion_ns(timeus[-1]))
    print(f"  IMU companion-ns coverage: [{first_ns} .. {last_ns}]")
    print(f"  image window [1783605974565266432 .. 1783606532881061376] covered: "
          f"{first_ns <= 1783605974565266432 and last_ns >= 1783606532881061376}")


if __name__ == "__main__":
    _self_test()
