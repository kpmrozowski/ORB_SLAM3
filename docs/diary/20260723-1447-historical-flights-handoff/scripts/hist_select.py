#!/usr/bin/env python3
"""Emit selected_flights.csv (the RUNNABLE historical flights) from hist_scan.csv +
the converted dataset metadata in datasets-nora/.

Output: /media/kmro/datasets/dops/flights/keypoints_analysis/selected_flights.csv
"""
import csv
import json
import os

ROOT = "/home/kmro/praca/dev/orbslam3-eval"
SCAN_CSV = os.path.join(ROOT, "eval_out", "hist_scan.csv")
DATASETS_ROOT = os.path.join(ROOT, "datasets-nora")
OUT_CSV = "/media/kmro/datasets/dops/flights/keypoints_analysis/selected_flights.csv"


BINSEL_CSV = ("/media/kmro/datasets/dops/flights/bin_session_selection/"
              "final_selected_sessions.csv")


def main():
    with open(SCAN_CSV, newline="") as handle:
        runnable = [dict(row, selection_source="keypoints_summary")
                    for row in csv.DictReader(handle) if row["verdict"] == "RUNNABLE"]
    seen = {row["flight_key"] for row in runnable}
    if os.path.isfile(BINSEL_CSV):
        with open(BINSEL_CSV, newline="") as handle:
            for row in csv.DictReader(handle):
                if row["final_selected"] != "True" or row["image_session_dir"] in seen:
                    continue
                source = row["image_source"]
                session_path = (source.split("::")[0] if "::" in source
                                else os.path.dirname(source))
                runnable.append(dict(
                    flight_key=row["image_session_dir"], session_ids=row["session_id"],
                    date="", drone=row["session_id"].split("_", 1)[1],
                    image_source="binsel", session_path=session_path,
                    bin=row["bin_path"], gps1_min_dist_m=row["gps0_min_dist_m"],
                    n_frames=row["num_images"], selection_source="bin_session_selection"))
    out_rows = []
    for row in sorted(runnable, key=lambda item: item["flight_key"]):
        meta_path = os.path.join(DATASETS_ROOT, row["flight_key"], "dataset_meta.json")
        meta = {}
        if os.path.isfile(meta_path):
            with open(meta_path) as handle:
                meta = json.load(handle)
        image_size = meta.get("image_size", ["", ""])
        out_rows.append({
            "flight_key": row["flight_key"],
            "keypoints_summary_sessions": row["session_ids"],
            "date": row["date"],
            "drone": row["drone"],
            "image_source": row["image_source"],
            "session_path": row.get("session_path", ""),
            "bin": meta.get("bin", row.get("bin", "")),
            "gps1_min_dist_m": row.get("gps1_min_dist_m", ""),
            "n_frames": meta.get("n_frames", row.get("n_frames", "")),
            "fps_median": f"{meta['fps_median']:.2f}" if "fps_median" in meta else "",
            "imu_hz_median": f"{meta['imu_hz_median']:.1f}" if "imu_hz_median" in meta else "",
            "image_width": image_size[0],
            "image_height": image_size[1],
            "eval_dataset": os.path.join(DATASETS_ROOT, row["flight_key"]),
            "orbslam3_config": meta.get("config", ""),
            "clock_bridge_residual_ms": (f"{meta['bridge']['median_abs_residual_ms']:.2f}"
                                         if "bridge" in meta else ""),
            "selection_source": row.get("selection_source", ""),
        })
    with open(OUT_CSV, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(out_rows[0].keys()))
        writer.writeheader()
        writer.writerows(out_rows)
    print(f"{len(out_rows)} selected flights -> {OUT_CSV}")
    for row in out_rows:
        print(f"  {row['flight_key']:<44} {row['image_width']}x{row['image_height']} "
              f"{row['n_frames']} frames  gps1min={row['gps1_min_dist_m']}m")


if __name__ == "__main__":
    main()
