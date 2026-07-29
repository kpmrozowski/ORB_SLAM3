#!/usr/bin/env python3
"""Convert a golem27 data-recorder session to EuRoC layout for ORB-SLAM3.

Data-recorder format (e.g. .../golem27_home/data-recorder/20260402_165608/):
    cam_0/<ns>.png                        IMX219 mono, native 1640x1232
    ardupilot_imu_<session>.csv           timestamp_ns, accel in g, gyro in deg/s

Output (EuRoC layout expected by mono_inertial_euroc):
    <out>/mav0/cam0/data/<ns>.png         resized to the calibrated 800x600
    <out>/mav0/imu0/data.csv              ns, gyro [rad/s], accel [m/s^2]
    <out>/cam0_times.txt                  one integer ns per kept frame

The IMU csv is trimmed to the frame window plus a margin, units converted to
SI, and optionally time-shifted (--imu-shift-s, kalibr convention: to undo
t_imu = t_cam + shift pass --imu-shift-s -<shift>). Use --imu-only to rewrite
just the IMU csv (e.g. after measuring the offset) without re-resizing images.
"""

import argparse
import csv
import math
import os
from multiprocessing import Pool

import cv2

GRAVITY = 9.80665
IMU_MARGIN_NS = 2_000_000_000


def find_imu_csv(src_dir):
    candidates = [name for name in os.listdir(src_dir)
                  if name.startswith("ardupilot_imu_") and name.endswith(".csv")]
    if len(candidates) != 1:
        raise SystemExit(f"expected exactly one ardupilot_imu_*.csv in {src_dir}, found {candidates}")
    return os.path.join(src_dir, candidates[0])


def list_frames(src_dir, start_ns, end_ns):
    stamps = sorted(int(name[:-4]) for name in os.listdir(os.path.join(src_dir, "cam_0"))
                    if name.endswith(".png"))
    return [stamp for stamp in stamps if start_ns <= stamp <= end_ns]


def resize_one(task):
    """Returns 'written', 'exists', or 'bad' (corrupt/unreadable source png)."""
    src_path, dst_path, width, height = task
    if os.path.exists(dst_path):
        return "exists"
    image = cv2.imread(src_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        return "bad"
    resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    cv2.imwrite(dst_path, resized)
    return "written"


def read_imu_si(imu_csv, shift_ns):
    """Read the ardupilot IMU csv, convert to SI, apply the time shift. Sorted by stamp."""
    import numpy as np

    deg_to_rad = math.pi / 180.0
    stamps, samples = [], []
    with open(imu_csv) as src:
        for row in csv.DictReader(src):
            stamps.append(int(row["timestamp_ns"]) + shift_ns)
            gyro = [float(row[f"gyro_{axis}_dps"]) * deg_to_rad for axis in "xyz"]
            accel = [float(row[f"accel_{axis}_g"]) * GRAVITY for axis in "xyz"]
            samples.append(gyro + accel)
    stamps = np.array(stamps, dtype=np.int64)
    samples = np.array(samples, dtype=float)
    order = np.argsort(stamps)
    return stamps[order], samples[order]


def resample_uniform(stamps, samples, rate_hz):
    """Uniform-grid linear resample over [first, last], filling dropout gaps.

    Filling a multi-second dropout with linear interpolation is a smooth-motion
    approximation, but it keeps every frame interval populated and bounds the
    preintegration step so ORB-SLAM3 stays numerically alive across the gap.
    """
    import numpy as np

    step_ns = int(round(1e9 / rate_hz))
    grid = np.arange(stamps[0], stamps[-1] + 1, step_ns, dtype=np.int64)
    resampled = np.column_stack([np.interp(grid, stamps, samples[:, axis]) for axis in range(samples.shape[1])])
    return grid, resampled


def write_imu(imu_csv, out_csv, shift_ns, window_lo_ns, window_hi_ns, fill_hz):
    import numpy as np

    stamps, samples = read_imu_si(imu_csv, shift_ns)
    if fill_hz > 0.0:
        stamps, samples = resample_uniform(stamps, samples, fill_hz)
    keep = (stamps >= window_lo_ns) & (stamps <= window_hi_ns)
    stamps, samples = stamps[keep], samples[keep]
    with open(out_csv, "w", newline="") as dst:
        dst.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                  "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        writer = csv.writer(dst)
        for stamp, sample in zip(stamps, samples):
            writer.writerow([int(stamp)] + [f"{value:.9f}" for value in sample])
    if len(stamps) == 0:
        raise SystemExit("no IMU rows in the requested window")
    return len(stamps), int(stamps[0]), int(stamps[-1])


def find_baro_csv(src_dir):
    """Locate the optional ardupilot_baro_*.csv. Returns its path or None (never raises on absence)."""
    candidates = [name for name in os.listdir(src_dir)
                  if name.startswith("ardupilot_baro_") and name.endswith(".csv")]
    if len(candidates) > 1:
        raise SystemExit(f"expected at most one ardupilot_baro_*.csv in {src_dir}, found {candidates}")
    return os.path.join(src_dir, candidates[0]) if candidates else None


def write_baro(baro_csv, out_csv, window_lo_ns, window_hi_ns):
    """Write a '#timestamp [ns],alt_m' altitude reference trimmed to [lo, hi] (same window as the IMU).

    Two-column layout read by eval/validate_nora.py::load_reference (barometer vertical-ATE).
    Stamps are kept on the native boot-monotonic clock (no rebase). Returns (rows, alt_min, alt_max).
    """
    stamps, altitudes = [], []
    with open(baro_csv) as src:
        for row in csv.DictReader(src):
            stamp = int(row["timestamp_ns"])
            if window_lo_ns <= stamp <= window_hi_ns:
                stamps.append(stamp)
                altitudes.append(float(row["altitude_m"]))
    order = sorted(range(len(stamps)), key=lambda index: stamps[index])
    with open(out_csv, "w", newline="") as dst:
        dst.write("#timestamp [ns],alt_m\n")
        writer = csv.writer(dst)
        for index in order:
            writer.writerow([stamps[index], f"{altitudes[index]:.6f}"])
    if not stamps:
        return 0, float("nan"), float("nan")
    return len(stamps), min(altitudes), max(altitudes)


def symlink_frames(src_dir, cam_dir, frames):
    """Symlink native cam_0/<ns>.png into cam_dir/<ns>.png at full resolution (no resize, no rename).

    The absolute source is linked so the dataset works from any cwd; zero-byte (corrupt) source PNGs
    are skipped. Returns (kept_frames, skipped_empty_frames).
    """
    kept, skipped_empty = [], []
    for stamp in frames:
        src_path = os.path.abspath(os.path.join(src_dir, "cam_0", f"{stamp}.png"))
        if os.path.getsize(src_path) == 0:
            skipped_empty.append(stamp)
            continue
        dst_path = os.path.join(cam_dir, f"{stamp}.png")
        if os.path.lexists(dst_path):
            os.remove(dst_path)
        os.symlink(src_path, dst_path)
        kept.append(stamp)
    return kept, skipped_empty


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("src", help="data-recorder session dir (cam_0/ + ardupilot_imu_*.csv)")
    parser.add_argument("out", help="output EuRoC dataset dir")
    parser.add_argument("--start-ns", type=int, default=0, help="first frame timestamp to keep")
    parser.add_argument("--end-ns", type=int, default=2**63 - 1, help="last frame timestamp to keep")
    parser.add_argument("--imu-shift-s", type=float, default=0.0,
                        help="seconds ADDED to IMU stamps (kalibr: pass -timeshift_cam_imu)")
    parser.add_argument("--fill-imu-hz", type=float, default=0.0,
                        help="resample IMU onto a uniform grid at this rate, linearly filling "
                             "dropout gaps (0=off; ardupilot data-recorder logs need it, e.g. 200)")
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=600)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--native-symlink", action="store_true",
                        help="skip resizing; symlink each kept native cam_0/<ns>.png (same name) instead")
    parser.add_argument("--imu-only", action="store_true", help="rewrite imu0/data.csv only")
    args = parser.parse_args()

    imu_dir = os.path.join(args.out, "mav0", "imu0")
    cam_dir = os.path.join(args.out, "mav0", "cam0", "data")
    os.makedirs(imu_dir, exist_ok=True)
    os.makedirs(cam_dir, exist_ok=True)

    # Process IMU first so frames can be trimmed to actual IMU coverage (frames
    # outside [imu_lo, imu_hi] have no measurements to preintegrate -> crashes).
    shift_ns = round(args.imu_shift_s * 1e9)
    window_lo_ns = args.start_ns - IMU_MARGIN_NS
    window_hi_ns = args.end_ns + IMU_MARGIN_NS
    kept, imu_lo, imu_hi = write_imu(find_imu_csv(args.src), os.path.join(imu_dir, "data.csv"),
                                     shift_ns, window_lo_ns, window_hi_ns, args.fill_imu_hz)
    fill_note = f"filled@{args.fill_imu_hz:.0f}Hz" if args.fill_imu_hz > 0 else "raw (gaps kept)"
    print(f"imu: {kept} rows (SI, shift {shift_ns / 1e6:+.2f} ms, {fill_note}) "
          f"span [{imu_lo}, {imu_hi}] -> {imu_dir}/data.csv")

    # Barometer sidecar (vertical-ATE reference), trimmed to the same window as the IMU.
    baro_csv = find_baro_csv(args.src)
    if baro_csv is not None:
        baro_out = os.path.join(args.out, "ref_baro.csv")
        baro_rows, alt_min, alt_max = write_baro(baro_csv, baro_out, window_lo_ns, window_hi_ns)
        print(f"baro: {baro_rows} rows, altitude [{alt_min:.2f}, {alt_max:.2f}] m -> {baro_out}")

    if not args.imu_only:
        # Bracket every frame by IMU: keep frames strictly inside IMU coverage.
        frames = [stamp for stamp in list_frames(args.src, args.start_ns, args.end_ns)
                  if imu_lo <= stamp <= imu_hi]
        if not frames:
            raise SystemExit("no frames within IMU coverage")
        if args.native_symlink:
            frames, skipped_empty = symlink_frames(args.src, cam_dir, frames)
            print(f"images: {len(frames)} native symlinks, "
                  f"{len(skipped_empty)} zero-byte SKIPPED {skipped_empty} -> {cam_dir}")
        else:
            tasks = [(os.path.join(args.src, "cam_0", f"{stamp}.png"),
                      os.path.join(cam_dir, f"{stamp}.png"), args.width, args.height)
                     for stamp in frames]
            with Pool(args.jobs) as pool:
                results = pool.map(resize_one, tasks, chunksize=32)
            bad = [stamp for stamp, result in zip(frames, results) if result == "bad"]
            frames = [stamp for stamp, result in zip(frames, results) if result != "bad"]
            print(f"images: {results.count('written')} resized, {results.count('exists')} already present, "
                  f"{len(bad)} corrupt SKIPPED {bad} -> {cam_dir}")
        if not frames:
            raise SystemExit("no frames left after image processing")
        with open(os.path.join(args.out, "cam0_times.txt"), "w") as handle:
            handle.write("\n".join(str(stamp) for stamp in frames) + "\n")
        span_s = (frames[-1] - frames[0]) / 1e9
        print(f"frames: {len(frames)} over {span_s:.1f} s ({len(frames) / span_s:.1f} fps), "
              f"window [{frames[0]}, {frames[-1]}]")


if __name__ == "__main__":
    main()
