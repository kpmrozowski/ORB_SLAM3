#!/usr/bin/env python3
"""Convert the golem27 'circle' drone dataset to EuRoC layout for ORB-SLAM3.

Parameterized rewrite of convert_to_euroc.py. Same core logic (symlink the already
800x600 grayscale PNGs, gyro-zero interior repair by linear interpolation from the
timestamp-nearest non-zero neighbours, EuRoC output layout), with two changes:

1. WINDOW: keep data with timestamp >= --start-ns through --end-ns (default = end of
   the recording). The old fixed T_END landing-trim is NOT applied. IMU is emitted from
   start - 2 s (IMU_MARGIN_NS) so ORB-SLAM3 has pre-first-frame IMU context.

2. REBASE (like convert_nora_to_euroc.py): every emitted timestamp is rebased by
   --epoch-ns (default = --start-ns) i.e. subtracted. ORB-SLAM3's EuRoC loader parses
   each stamp into a double and multiplies by 1e-9; at absolute Unix-ns magnitude
   (~1.775e18) a double's ULP is ~512 ns, which jitters/collapses IMU dt and breaks
   preintegration. Rebased (~1e10) the ULP is sub-nanosecond. Image filenames,
   cam0_times.txt and the IMU csv stamps are all `t - epoch`; the epoch is recorded in
   dataset_meta.json (add it back to recover Unix time).

Output (EuRoC layout):
    <out>/mav0/cam0/data/<rebased_ns>.png   symlink to the original (source untouched)
    <out>/mav0/imu0/data.csv                 rebased-ns, gyro[rad/s], accel[m/s^2]
    <out>/cam0_times.txt                      one rebased-ns integer per kept frame
    <out>/dataset_meta.json                   window, epoch, counts, source paths
"""

import argparse
import csv
import json
import os

import numpy as np

SRC_DEFAULT = "/home/kmro/praca/dev/datasets/droneops-calibrations/golem27_home/20260402-001-circle-20260402"
OUT_DEFAULT = "/home/kmro/praca/dev/orbslam3-eval/dataset/circle_move"
START_NS_DEFAULT = 1775119067080676352
IMU_MARGIN_NS = 2_000_000_000
GYRO_ZERO_EPS = 1e-9
ACCEL_SPIKE_THRESH = 40.0  # m/s^2, |a| per-axis


def load_imu(src):
    """Load imu0.csv -> (ts[int64], gyro[rad/s], accel[m/s^2]). EuRoC order gyro then accel."""
    timestamps, gyro, accel = [], [], []
    with open(os.path.join(src, "imu0.csv")) as handle:
        reader = csv.reader(handle)
        next(reader)  # header: timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z
        for row in reader:
            timestamps.append(int(row[0]))
            gyro.append([float(row[1]), float(row[2]), float(row[3])])
            accel.append([float(row[4]), float(row[5]), float(row[6])])
    return np.array(timestamps, dtype=np.int64), np.array(gyro), np.array(accel)


def repair_isolated_zeros(timestamps, gyro):
    """Linearly interpolate gyro on zero rows from their timestamp-nearest non-zero
    neighbours. Rows whose only non-zero neighbour is on one side (leading/trailing
    zero runs of the window) have no bracketing pair and are left as zero."""
    norm = np.linalg.norm(gyro, axis=1)
    zero_rows = np.where(norm < GYRO_ZERO_EPS)[0]
    good = norm >= GYRO_ZERO_EPS
    repaired = 0
    for zero_idx in zero_rows:
        prev = zero_idx - 1
        while prev >= 0 and not good[prev]:
            prev -= 1
        nxt = zero_idx + 1
        while nxt < len(timestamps) and not good[nxt]:
            nxt += 1
        if prev < 0 or nxt >= len(timestamps):
            continue
        frac = (timestamps[zero_idx] - timestamps[prev]) / (timestamps[nxt] - timestamps[prev])
        gyro[zero_idx] = gyro[prev] + frac * (gyro[nxt] - gyro[prev])
        repaired += 1
    return repaired


def leading_trailing_zero_runs(gyro):
    """(leading, trailing) counts of contiguous gyro-zero rows at the window edges."""
    zero = np.linalg.norm(gyro, axis=1) < GYRO_ZERO_EPS
    leading = 0
    while leading < len(zero) and zero[leading]:
        leading += 1
    trailing = 0
    idx = len(zero) - 1
    while idx >= 0 and zero[idx]:
        trailing += 1
        idx -= 1
    return leading, trailing


def list_frame_stamps(src, start_ns, end_ns):
    """Sorted Unix-ns stamps of every cam0 PNG in [start_ns, end_ns] with a non-empty
    source file. Zero-byte PNGs (camera write failures) can't be repaired (read-only
    source) so they are skipped and counted; the continuous IMU bridges the gap."""
    image_dir = os.path.join(src, "cam0")
    stamps = sorted(int(name[:-4]) for name in os.listdir(image_dir)
                    if name.endswith(".png") and name[:-4].isdigit())
    in_window = [stamp for stamp in stamps if start_ns <= stamp <= end_ns]
    kept = [stamp for stamp in in_window
            if os.path.getsize(os.path.join(image_dir, f"{stamp}.png")) > 0]
    return kept, len(in_window) - len(kept)


def link_frames(src, cam_dir, frames, epoch_ns):
    """Symlink each original (Unix-ns-named) PNG to its rebased-ns filename so the stem
    matches the rebased cam0_times.txt entry the loader keys on."""
    image_dir = os.path.join(src, "cam0")
    for stamp in frames:
        source_png = os.path.join(image_dir, f"{stamp}.png")
        dst = os.path.join(cam_dir, f"{stamp - epoch_ns}.png")
        if os.path.lexists(dst):
            os.remove(dst)
        os.symlink(source_png, dst)
    return len(frames)


def write_imu_csv(out_csv, timestamps, gyro, accel, epoch_ns):
    with open(out_csv, "w") as handle:
        handle.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                     "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for stamp, gyro_row, accel_row in zip(timestamps, gyro, accel):
            handle.write(f"{int(stamp) - epoch_ns},"
                         f"{gyro_row[0]:.9g},{gyro_row[1]:.9g},{gyro_row[2]:.9g},"
                         f"{accel_row[0]:.9g},{accel_row[1]:.9g},{accel_row[2]:.9g}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--src", default=SRC_DEFAULT, help="source circle dataset dir")
    parser.add_argument("--out", default=OUT_DEFAULT, help="output EuRoC dataset dir")
    parser.add_argument("--start-ns", type=int, default=START_NS_DEFAULT,
                        help="keep data with timestamp >= this (Unix ns)")
    parser.add_argument("--end-ns", type=int, default=None,
                        help="keep data with timestamp <= this (default: end of recording)")
    parser.add_argument("--epoch-ns", type=int, default=None,
                        help="rebase epoch subtracted from every emitted stamp (default: --start-ns)")
    args = parser.parse_args()

    epoch_ns = args.start_ns if args.epoch_ns is None else args.epoch_ns
    timestamps, gyro, accel = load_imu(args.src)
    end_ns = int(timestamps[-1]) if args.end_ns is None else args.end_ns
    print(f"start_ns {args.start_ns} ({args.start_ns / 1e9:.3f})  "
          f"end_ns {end_ns} ({end_ns / 1e9:.3f})  epoch_ns {epoch_ns}")

    # --- IMU: [start - margin, end], then repair interior gyro-zeros ---
    imu_lo = args.start_ns - IMU_MARGIN_NS
    sel = (timestamps >= imu_lo) & (timestamps <= end_ns)
    imu_ts, imu_gyro, imu_accel = timestamps[sel].copy(), gyro[sel].copy(), accel[sel].copy()
    zero_before = int((np.linalg.norm(imu_gyro, axis=1) < GYRO_ZERO_EPS).sum())
    lead_run, trail_run = leading_trailing_zero_runs(imu_gyro)
    repaired = repair_isolated_zeros(imu_ts, imu_gyro)
    zero_after = int((np.linalg.norm(imu_gyro, axis=1) < GYRO_ZERO_EPS).sum())
    imu_dt = np.diff(imu_ts)
    imu_hz = 1e9 / float(np.median(imu_dt))
    print(f"IMU rows: {len(imu_ts)} (with {IMU_MARGIN_NS / 1e9:.0f}s pre-start margin), "
          f"median {1e9 / imu_hz / 1e6:.4f} ms -> {imu_hz:.4f} Hz")
    print(f"  gyro-zero rows before repair: {zero_before}  repaired (bracketed): {repaired}  "
          f"still zero: {zero_after}")
    print(f"  leading gyro-zero run: {lead_run}  trailing gyro-zero run: {trail_run} (both kept as-is)")

    accel_norm_axis = np.abs(imu_accel).max(axis=1)
    spike_mask = accel_norm_axis > ACCEL_SPIKE_THRESH
    spike_count = int(spike_mask.sum())
    first_spike_ns = int(imu_ts[np.where(spike_mask)[0][0]]) if spike_count else None
    max_abs_accel = float(accel_norm_axis.max())
    print(f"  accel spikes |a|>{ACCEL_SPIKE_THRESH:.0f} m/s^2: {spike_count}  "
          f"first {'%d (%.3f)' % (first_spike_ns, first_spike_ns / 1e9) if first_spike_ns else 'none'}  "
          f"max|a| {max_abs_accel:.2f}")

    # --- Frames: [start, end] within IMU coverage ---
    imu_first, imu_last = int(imu_ts[0]), int(imu_ts[-1])
    frames_all, zero_byte = list_frame_stamps(args.src, args.start_ns, end_ns)
    frames = [stamp for stamp in frames_all if imu_first <= stamp <= imu_last]
    if not frames:
        raise SystemExit("no frames within IMU coverage - check --start-ns/--end-ns")
    span_s = (frames[-1] - frames[0]) / 1e9
    frame_dt = np.diff(np.array(frames, dtype=np.int64))
    fps_median = 1e9 / float(np.median(frame_dt))
    print(f"frames: {len(frames)} span {span_s:.3f} s  median dt {np.median(frame_dt) / 1e6:.3f} ms "
          f"-> {fps_median:.4f} fps (mean {len(frames) / span_s:.4f})  zero-byte skipped: {zero_byte}")
    if len(frames) < 50:
        print(f"  *** WARNING: only {len(frames)} frames (<50) - window likely too short/degenerate ***")

    # --- write layout ---
    cam_dir = os.path.join(args.out, "mav0", "cam0", "data")
    imu_dir = os.path.join(args.out, "mav0", "imu0")
    os.makedirs(cam_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)
    for existing in os.listdir(cam_dir):
        os.remove(os.path.join(cam_dir, existing))

    link_frames(args.src, cam_dir, frames, epoch_ns)
    with open(os.path.join(args.out, "cam0_times.txt"), "w") as handle:
        handle.write("\n".join(str(stamp - epoch_ns) for stamp in frames) + "\n")
    write_imu_csv(os.path.join(imu_dir, "data.csv"), imu_ts, imu_gyro, imu_accel, epoch_ns)

    meta = {
        "src": os.path.abspath(args.src),
        "start_ns": args.start_ns,
        "end_ns": end_ns,
        "epoch_ns": epoch_ns,
        "imu_margin_ns": IMU_MARGIN_NS,
        "n_frames": len(frames),
        "n_imu": len(imu_ts),
        "frame_first_ns": frames[0],
        "frame_last_ns": frames[-1],
        "frame_span_s": span_s,
        "fps_median": fps_median,
        "imu_first_ns": imu_first,
        "imu_last_ns": imu_last,
        "imu_hz_median": imu_hz,
        "gyro_zero_before_repair": zero_before,
        "gyro_zero_repaired": repaired,
        "gyro_zero_remaining": zero_after,
        "gyro_zero_leading_run": lead_run,
        "gyro_zero_trailing_run": trail_run,
        "accel_spike_thresh": ACCEL_SPIKE_THRESH,
        "accel_spike_count": spike_count,
        "accel_first_spike_ns": first_spike_ns,
        "accel_max_abs": max_abs_accel,
        "zero_byte_frames_skipped": zero_byte,
        "clock": "Unix nanoseconds REBASED by epoch_ns (subtract to compare; add back for Unix)",
    }
    with open(os.path.join(args.out, "dataset_meta.json"), "w") as handle:
        json.dump(meta, handle, indent=2)
    print(f"wrote EuRoC layout to {args.out}  (rebased epoch {epoch_ns})")


if __name__ == "__main__":
    main()
