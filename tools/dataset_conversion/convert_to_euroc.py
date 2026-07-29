#!/usr/bin/env python3
"""Convert golem27 'circle' drone dataset to EuRoC layout for ORB-SLAM3.

- Images are already 800x600 grayscale and match the 800x600 kalibr calibration,
  so we symlink them (no resize needed).
- Trims the takeoff (leading intermittent gyro-zero rows) and the hard-landing
  (trailing gyro-zero rows + accel spikes to -63 m/s^2).
- Repairs the handful of isolated gyro-zero rows inside the flight by linear
  interpolation from timestamp-nearest non-zero neighbours.
- Writes:
    <out>/mav0/cam0/data/<ns>.png       (symlinks)
    <out>/mav0/imu0/data.csv            (# header, ns, gyro xyz, accel xyz)
    <out>/cam0_times.txt                (one ns integer per frame)
"""
import os
import csv
import numpy as np

SRC = "/home/kmro/praca/dev/datasets/droneops-calibrations/golem27_home/20260402-001-circle-20260402"
OUT = "/home/kmro/praca/dev/orbslam3-eval/dataset/circle"

# Window end: exclude the landing accel spikes (which start at ts 1775119068.541e9).
T_END_NS = 1775119066000000000  # ~2.5 s before the first landing spike

def load_imu():
    ts, gyro, acc = [], [], []
    with open(os.path.join(SRC, "imu0.csv")) as handle:
        reader = csv.reader(handle)
        next(reader)
        for row in reader:
            ts.append(int(row[0]))
            gyro.append([float(row[1]), float(row[2]), float(row[3])])
            acc.append([float(row[4]), float(row[5]), float(row[6])])
    return np.array(ts), np.array(gyro), np.array(acc)

def find_clean_start(ts, gyro):
    """First index of the large clean block (rolling zero-fraction < 2%)."""
    is_zero = (np.linalg.norm(gyro, axis=1) < 1e-9).astype(float)
    window = 201
    kernel = np.ones(window) / window
    roll = np.convolve(is_zero, kernel, mode="same")
    clean = roll < 0.02
    idx = np.where(clean)[0]
    splits = np.where(np.diff(idx) > 1)[0]
    runs = np.split(idx, splits + 1)
    best = max(runs, key=len)
    return int(best[0])

def repair_isolated_zeros(ts, gyro):
    """Linearly interpolate gyro on isolated zero rows from non-zero neighbours."""
    norm = np.linalg.norm(gyro, axis=1)
    zero_rows = np.where(norm < 1e-9)[0]
    good = norm >= 1e-9
    repaired = 0
    for zi in zero_rows:
        prev = zi - 1
        while prev >= 0 and not good[prev]:
            prev -= 1
        nxt = zi + 1
        while nxt < len(ts) and not good[nxt]:
            nxt += 1
        if prev < 0 or nxt >= len(ts):
            continue
        frac = (ts[zi] - ts[prev]) / (ts[nxt] - ts[prev])
        gyro[zi] = gyro[prev] + frac * (gyro[nxt] - gyro[prev])
        repaired += 1
    return repaired

def main():
    ts, gyro, acc = load_imu()
    start_idx = find_clean_start(ts, gyro)
    t_start = ts[start_idx]
    print(f"clean IMU start idx {start_idx} ts {t_start} ({t_start/1e9:.3f})")

    # IMU rows in [t_start, T_END + 100ms buffer]
    imu_hi = T_END_NS + 100_000_000
    sel = (ts >= t_start) & (ts <= imu_hi)
    ts_w, gyro_w, acc_w = ts[sel].copy(), gyro[sel].copy(), acc[sel].copy()
    n_rep = repair_isolated_zeros(ts_w, gyro_w)
    print(f"IMU rows: {len(ts_w)}, repaired isolated gyro-zeros: {n_rep}")
    remaining_zero = int((np.linalg.norm(gyro_w, axis=1) < 1e-9).sum())
    print(f"remaining gyro-zero rows in window: {remaining_zero}")
    spikes = int((np.abs(acc_w).max(axis=1) > 30).sum())
    print(f"accel spikes (>30 m/s^2) remaining in window: {spikes}")

    # camera frames in [t_start, T_END]
    cam = sorted(int(f[:-4]) for f in os.listdir(os.path.join(SRC, "cam0")) if f.endswith(".png"))
    cam = np.array(cam)
    cam_sel = cam[(cam >= t_start) & (cam <= T_END_NS)]
    print(f"camera frames: {len(cam_sel)} span {(cam_sel[-1]-cam_sel[0])/1e9:.2f}s "
          f"({len(cam_sel)/((cam_sel[-1]-cam_sel[0])/1e9):.2f} FPS)")

    # write layout
    data_dir = os.path.join(OUT, "mav0", "cam0", "data")
    imu_dir = os.path.join(OUT, "mav0", "imu0")
    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)
    # clear old symlinks
    for existing in os.listdir(data_dir):
        os.remove(os.path.join(data_dir, existing))

    with open(os.path.join(OUT, "cam0_times.txt"), "w") as tf:
        for stamp in cam_sel:
            src_png = os.path.join(SRC, "cam0", f"{stamp}.png")
            os.symlink(src_png, os.path.join(data_dir, f"{stamp}.png"))
            tf.write(f"{stamp}\n")

    with open(os.path.join(imu_dir, "data.csv"), "w") as imu_f:
        imu_f.write("#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n")
        for stamp, gvec, avec in zip(ts_w, gyro_w, acc_w):
            imu_f.write(f"{int(stamp)},{gvec[0]:.9g},{gvec[1]:.9g},{gvec[2]:.9g},"
                        f"{avec[0]:.9g},{avec[1]:.9g},{avec[2]:.9g}\n")
    print(f"wrote EuRoC layout to {OUT}")

if __name__ == "__main__":
    main()
