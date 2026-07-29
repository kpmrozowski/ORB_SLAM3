#!/usr/bin/env python3
"""Emit ref_mag.csv for every converted historical dataset (datasets-nora/*).

MagFusion consumes "#timestamp [ns],mx,my,mz": the Earth field as a UNIT vector in the
SLAM BODY (IMU) frame at rebased-ns stamps. For these flights the SLAM body frame IS the
FC body frame (RISI preintegrated IMU), and the BIN MAG instance-0 values are already
FC-calibrated body-frame field - so no extra vehicle->IMU alignment is needed
(unlike the 538/542 export_mag_csv.py path).
"""
import json
import os
import re

import numpy as np

ROOT = "/home/kmro/praca/dev/orbslam3-eval"
BINCACHE_ROOT = os.path.join(ROOT, "eval_out", "bincache")
DATASETS_ROOT = os.path.join(ROOT, "datasets-nora")


def export_one(flight_key):
    dataset_dir = os.path.join(DATASETS_ROOT, flight_key)
    meta_path = os.path.join(dataset_dir, "dataset_meta.json")
    if not os.path.isfile(meta_path):
        return "no meta"
    with open(meta_path) as handle:
        meta = json.load(handle)
    cache_dir = os.path.join(
        BINCACHE_ROOT,
        re.sub(r"[^A-Za-z0-9]", "_", meta["bin"]) + f"_{os.path.getsize(meta['bin'])}")
    mag_npz = os.path.join(cache_dir, "hist_mag.npz")
    if not os.path.isfile(mag_npz):
        return "no mag cache"
    mag = dict(np.load(mag_npz))
    keep = mag["inst"] == 0
    if keep.sum() < 50:
        return f"only {int(keep.sum())} inst-0 rows"
    bridge = meta["bridge"]
    stamps_ns = (np.round(bridge["slope"] * 1000.0
                          * mag["timeus"][keep].astype(np.float64)).astype(np.int64)
                 + round(bridge["k_seconds"] * 1e9) - meta["epoch_ns"])
    field = np.column_stack([mag["magx"][keep], mag["magy"][keep], mag["magz"][keep]])
    order = np.argsort(stamps_ns)
    stamps_ns, field = stamps_ns[order], field[order].astype(np.float64)
    norms = np.linalg.norm(field, axis=1)
    valid = norms > 1e-6
    stamps_ns, field, norms = stamps_ns[valid], field[valid], norms[valid]
    field /= norms[:, None]

    with open(os.path.join(dataset_dir, "cam0_times.txt")) as handle:
        frame_stamps = [int(line) for line in handle if line.strip()]
    margin_ns = 5_000_000_000
    window = (stamps_ns >= frame_stamps[0] - margin_ns) & (stamps_ns <= frame_stamps[-1] + margin_ns)
    stamps_ns, field = stamps_ns[window], field[window]
    if len(stamps_ns) < 50:
        return "no rows in dataset window"
    out_path = os.path.join(dataset_dir, "ref_mag.csv")
    with open(out_path, "w") as handle:
        handle.write("#timestamp [ns],mx,my,mz\n")
        for stamp, vector in zip(stamps_ns, field):
            handle.write(f"{int(stamp)},{vector[0]:.6f},{vector[1]:.6f},{vector[2]:.6f}\n")
    return f"{len(stamps_ns)} rows"


def main():
    for flight_key in sorted(os.listdir(DATASETS_ROOT)):
        if os.path.isdir(os.path.join(DATASETS_ROOT, flight_key)):
            print(f"{flight_key:<46} {export_one(flight_key)}")


if __name__ == "__main__":
    main()
