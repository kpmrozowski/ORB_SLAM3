#!/usr/bin/env python3
"""Convert a NORA golem27 session (high_res_images + ArduPilot BIN RISI IMU) to the
EuRoC layout consumed by ORB-SLAM3 mono / mono_inertial_euroc.

NORA session format:
    <session>/high_res_images/<companion_unix_ns>.png     mono, 1640x1232 (grayscale)
    <session>/high_res_images/high_res_frames.csv         timestamp_ns, exposure_us, bytes
IMU lives in an ArduPilot BIN as preintegrated RISI messages, pre-parsed and cached
by nora_20260709/lib/nora_bin.py (RISI DA/DADT -> gyro rad/s, DV/DVDT -> accel m/s^2),
with the verified companion-Unix-ns <-> flight-controller TimeUS clock map.

Output (EuRoC layout):
    <out>/mav0/cam0/data/<companion_ns>.png    symlink to the original (nothing modified)
    <out>/mav0/imu0/data.csv                   ns, gyro[rad/s], accel[m/s^2]
    <out>/cam0_times.txt                        one integer companion-ns per kept frame
    <out>/dataset_meta.json                     window, offsets, source paths
Optional sidecar references for scale validation (Unix-ns stamped):
    <out>/ref_baro.csv (t_ns, alt_m), ref_rfnd.csv (t_ns, dist_m)

All emitted timestamps are REBASED to an epoch (default = --start-ns) i.e. nanoseconds
since the window start. This is required for numerical safety: ORB-SLAM3's EuRoC loader
parses each stamp into a double and divides by 1e9, and at absolute Unix-ns magnitude
(~1.78e18) a double's ULP is ~512 ns, which jitters/collapses IMU dt and breaks
preintegration. Rebased (~5.6e11) the ULP is sub-nanosecond. The epoch is recorded in
dataset_meta.json and applied to the sidecar references too, so trajectory output, baro,
rangefinder and GPS all share one rebased clock; add the epoch back to recover Unix time.
The cam-imu timeshift follows the kalibr convention: to undo t_imu = t_cam +
timeshift_cam_imu, pass --imu-shift-s -<timeshift_cam_imu> (default calibration.yaml, -0.038675 s).
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "nora_20260709", "lib"))
import nora_bin  # noqa: E402

IMU_MARGIN_NS = 2_000_000_000


def list_frame_stamps(session_dir, start_ns, end_ns):
    """Companion-ns stamps of every high_res PNG inside [start_ns, end_ns], sorted.
    Zero-byte source PNGs (camera write failures, e.g. 56 frames in the 542 session) are
    skipped: the source is read-only so they cannot be repaired, and the continuous IMU
    bridges the resulting gaps exactly like ordinary camera frame drops."""
    image_dir = os.path.join(session_dir, "high_res_images")
    stamps = sorted(int(name[:-4]) for name in os.listdir(image_dir)
                    if name.endswith(".png") and name[:-4].isdigit())
    in_window = [stamp for stamp in stamps if start_ns <= stamp <= end_ns]
    kept = [stamp for stamp in in_window
            if os.path.getsize(os.path.join(image_dir, f"{stamp}.png")) > 0]
    dropped = len(in_window) - len(kept)
    if dropped:
        print(f"list_frame_stamps: skipped {dropped} zero-byte source PNGs")
    return kept


def resample_uniform(stamps_ns, samples, rate_hz):
    """Uniform-grid linear resample over [first, last]; fills any IMU dropout gaps so
    preintegration steps stay bounded. RISI is normally continuous across camera frame
    drops, so this is only needed if the BIN itself has IMU gaps."""
    step_ns = int(round(1e9 / rate_hz))
    grid = np.arange(stamps_ns[0], stamps_ns[-1] + 1, step_ns, dtype=np.int64)
    resampled = np.column_stack([np.interp(grid, stamps_ns, samples[:, axis])
                                 for axis in range(samples.shape[1])])
    return grid, resampled


def build_imu(cached, shift_ns, window_lo_ns, window_hi_ns, fill_hz):
    """From the loaded BIN cache, convert RISI to SI gyro+accel, stamp on the companion
    Unix-ns clock, apply the cam-imu shift, sort, optionally gap-fill, and trim to the window."""
    timeus, gyro_xyz, accel_xyz = nora_bin.risi_to_imu(cached["risi"])
    stamps_ns = nora_bin.timeus_to_companion_ns(timeus) + shift_ns
    samples = np.column_stack([gyro_xyz, accel_xyz])  # EuRoC order: gyro then accel

    order = np.argsort(stamps_ns)
    stamps_ns, samples = stamps_ns[order].astype(np.int64), samples[order]
    # Drop exact-duplicate stamps (preintegration rejects non-increasing dt).
    keep_unique = np.concatenate(([True], np.diff(stamps_ns) > 0))
    stamps_ns, samples = stamps_ns[keep_unique], samples[keep_unique]

    if fill_hz > 0.0:
        stamps_ns, samples = resample_uniform(stamps_ns, samples, fill_hz)

    in_window = (stamps_ns >= window_lo_ns) & (stamps_ns <= window_hi_ns)
    return stamps_ns[in_window], samples[in_window]


def write_imu_csv(out_csv, stamps_ns, samples, epoch_ns):
    with open(out_csv, "w", newline="") as handle:
        handle.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                     "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        writer = csv.writer(handle)
        for stamp, sample in zip(stamps_ns, samples):
            writer.writerow([int(stamp) - epoch_ns] + [f"{value:.9f}" for value in sample])


def write_reference(out_csv, stamps_ns, values, header, epoch_ns):
    with open(out_csv, "w", newline="") as handle:
        handle.write(header + "\n")
        writer = csv.writer(handle)
        for stamp, value in zip(stamps_ns, values):
            writer.writerow([int(stamp) - epoch_ns, f"{float(value):.6f}"])


def link_frames(session_dir, cam_dir, frames, epoch_ns, use_symlink):
    """Symlink each original (absolute-Unix-ns-named) PNG to a rebased-ns filename so the
    stem matches the rebased cam0_times.txt entry the loader keys on."""
    image_dir = os.path.join(session_dir, "high_res_images")
    linked = 0
    for stamp in frames:
        src = os.path.join(image_dir, f"{stamp}.png")
        dst = os.path.join(cam_dir, f"{stamp - epoch_ns}.png")
        if os.path.lexists(dst):
            os.remove(dst)
        if use_symlink:
            os.symlink(src, dst)
        else:
            import shutil
            shutil.copy(src, dst)
        linked += 1
    return linked


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session", required=True, help="NORA session dir (has high_res_images/)")
    parser.add_argument("--cache", required=True, help="BIN cache dir (nora_bin npz files)")
    parser.add_argument("--out", required=True, help="output EuRoC dataset dir")
    parser.add_argument("--start-ns", type=int, required=True)
    parser.add_argument("--end-ns", type=int, required=True)
    parser.add_argument("--epoch-ns", type=int, default=None,
                        help="rebase epoch subtracted from every emitted stamp (default: --start-ns)")
    parser.add_argument("--imu-shift-s", type=float, default=-0.038675,
                        help="seconds ADDED to IMU stamps (kalibr: -timeshift_cam_imu)")
    parser.add_argument("--fill-imu-hz", type=float, default=0.0,
                        help="uniform-resample IMU at this rate, filling gaps (0=off)")
    parser.add_argument("--copy", action="store_true", help="copy frames instead of symlink")
    parser.add_argument("--imu-only", action="store_true", help="rewrite imu0/data.csv only")
    args = parser.parse_args()

    epoch_ns = args.start_ns if args.epoch_ns is None else args.epoch_ns
    imu_dir = os.path.join(args.out, "mav0", "imu0")
    cam_dir = os.path.join(args.out, "mav0", "cam0", "data")
    os.makedirs(imu_dir, exist_ok=True)
    os.makedirs(cam_dir, exist_ok=True)

    shift_ns = round(args.imu_shift_s * 1e9)
    cached = nora_bin.load_cached(args.cache)
    stamps_ns, samples = build_imu(cached, shift_ns,
                                   args.start_ns - IMU_MARGIN_NS,
                                   args.end_ns + IMU_MARGIN_NS, args.fill_imu_hz)
    if len(stamps_ns) == 0:
        raise SystemExit("no IMU rows in the requested window - check the sync map")
    write_imu_csv(os.path.join(imu_dir, "data.csv"), stamps_ns, samples, epoch_ns)
    imu_lo, imu_hi = int(stamps_ns[0]), int(stamps_ns[-1])
    median_dt_ms = float(np.median(np.diff(stamps_ns))) / 1e6
    fill_note = f"filled@{args.fill_imu_hz:.0f}Hz" if args.fill_imu_hz > 0 else "raw"
    print(f"imu: {len(stamps_ns)} rows (SI, shift {shift_ns / 1e6:+.2f} ms, {fill_note}, "
          f"~{1000.0 / median_dt_ms:.0f} Hz) span [{imu_lo}, {imu_hi}]")

    # Sidecar references for scale validation (rebased-ns stamped), best-effort.
    for name, accessor, arrays_key, header in [
        ("ref_baro.csv", nora_bin.baro_alt, "baro", "#timestamp [ns],alt_m"),
        ("ref_rfnd.csv", nora_bin.rfnd_dist, "rfnd", "#timestamp [ns],dist_m"),
    ]:
        try:
            ref_ns, ref_values = accessor(cached[arrays_key])
            keep = (ref_ns >= imu_lo) & (ref_ns <= imu_hi)
            write_reference(os.path.join(args.out, name), ref_ns[keep], ref_values[keep], header, epoch_ns)
            print(f"ref: {name} {int(keep.sum())} rows")
        except Exception as error:  # noqa: BLE001 - reference export is optional
            print(f"ref: {name} skipped ({error})")

    if not args.imu_only:
        frames = [stamp for stamp in list_frame_stamps(args.session, args.start_ns, args.end_ns)
                  if imu_lo <= stamp <= imu_hi]
        if not frames:
            raise SystemExit("no frames within IMU coverage")
        linked = link_frames(args.session, cam_dir, frames, epoch_ns, use_symlink=not args.copy)
        with open(os.path.join(args.out, "cam0_times.txt"), "w") as handle:
            handle.write("\n".join(str(stamp - epoch_ns) for stamp in frames) + "\n")
        span_s = (frames[-1] - frames[0]) / 1e9
        print(f"frames: {linked} {'copied' if args.copy else 'symlinked'} over {span_s:.1f} s "
              f"({len(frames) / span_s:.1f} fps) window [{frames[0]}, {frames[-1]}] "
              f"rebased epoch {epoch_ns}")

        meta = {
            "session": os.path.abspath(args.session),
            "cache": os.path.abspath(args.cache),
            "start_ns": args.start_ns, "end_ns": args.end_ns,
            "epoch_ns": epoch_ns,
            "imu_shift_s": args.imu_shift_s, "fill_imu_hz": args.fill_imu_hz,
            "n_frames": len(frames), "n_imu": len(stamps_ns),
            "frame_first_ns": frames[0], "frame_last_ns": frames[-1],
            "imu_first_ns": imu_lo, "imu_last_ns": imu_hi,
            "clock": "companion Unix nanoseconds, REBASED by epoch_ns (subtract to compare, add back for Unix)",
        }
        with open(os.path.join(args.out, "dataset_meta.json"), "w") as handle:
            json.dump(meta, handle, indent=2)


if __name__ == "__main__":
    main()
