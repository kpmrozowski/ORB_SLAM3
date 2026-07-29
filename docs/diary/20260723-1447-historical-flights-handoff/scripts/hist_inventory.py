#!/usr/bin/env python3
"""Phase A inventory for the historical keypoints_summary.csv sessions.

For every session row: locate the session storage (plain dir vs .temp.zip), count
images, and list candidate ArduPilot .BIN logs under the same <date>/<drone> subtree.
Writes hist_inventory.csv next to keypoints_summary.csv reference copy in eval/out.
Filesystem-only: no BIN parsing here.
"""
import csv
import os
import re
import zipfile

FLIGHTS_ROOT = "/media/kmro/datasets/dops/flights"
SUMMARY_CSV = "/media/kmro/datasets/dops/flights/keypoints_analysis/keypoints_summary.csv"
OUT_CSV = "/home/kmro/praca/dev/orbslam3-eval/eval_out/hist_inventory.csv"


def find_bin_candidates(date_token, drone_token):
    """All .BIN files (ArduPilot dataflash) under flights/<date>/<drone>, sorted by size."""
    base = os.path.join(FLIGHTS_ROOT, date_token, drone_token)
    candidates = []
    if not os.path.isdir(base):
        return candidates
    for dirpath, _dirnames, filenames in os.walk(base):
        for filename in filenames:
            if filename.upper().endswith(".BIN"):
                full = os.path.join(dirpath, filename)
                try:
                    size = os.path.getsize(full)
                except OSError:
                    size = -1
                candidates.append((full, size))
    candidates.sort(key=lambda item: -item[1])
    return candidates


def session_storage(source_field, session_dir_field):
    """Resolve where the session lives. Returns (kind, session_path, images_hint)."""
    if ".temp.zip::" in source_field:
        zip_path = source_field.split("::")[0]
        if os.path.isfile(zip_path):
            try:
                with zipfile.ZipFile(zip_path) as archive:
                    names = archive.namelist()
                image_names = [name for name in names
                               if re.search(r"images/[^/]+\.(png|jpg)$", name)
                               and "additional" not in name]
                hdf_names = [name for name in names if name.endswith(".hdf")]
                return "zip", zip_path, len(image_names), len(hdf_names)
            except zipfile.BadZipFile:
                return "zip_bad", zip_path, 0, 0
        return "zip_missing", zip_path, 0, 0
    session_path = source_field
    for suffix in ("/keypoints/", "/keypoints"):
        if session_path.endswith(suffix):
            session_path = session_path[: -len(suffix)]
            break
    if os.path.isdir(session_path):
        image_dir = os.path.join(session_path, "images")
        hdf_dir = os.path.join(session_path, "hdf_logs")
        n_images = len(os.listdir(image_dir)) if os.path.isdir(image_dir) else 0
        n_hdf = len(os.listdir(hdf_dir)) if os.path.isdir(hdf_dir) else 0
        return "dir", session_path, n_images, n_hdf
    return "missing", session_path, 0, 0


def main():
    rows = []
    with open(SUMMARY_CSV, newline="") as handle:
        for record in csv.DictReader(handle):
            source = record["source"]
            match = re.search(r"/flights/([^/]+)/([^/]+)/", source)
            date_token, drone_token = (match.group(1), match.group(2)) if match else ("?", "?")
            kind, session_path, n_images, n_hdf = session_storage(source, record["session_dir"])
            candidates = find_bin_candidates(date_token, drone_token)
            rows.append({
                "session_id": record["session_id"],
                "session_dir": record["session_dir"],
                "date": date_token,
                "drone": drone_token,
                "storage": kind,
                "session_path": session_path,
                "n_images": n_images,
                "n_hdf": n_hdf,
                "n_bin": len(candidates),
                "bin_paths": ";".join(path for path, _size in candidates[:8]),
                "bin_sizes_mb": ";".join(f"{size / 1e6:.0f}" for _path, size in candidates[:8]),
            })
    os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
    with open(OUT_CSV, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    n_with_bin = sum(1 for row in rows if row["n_bin"] > 0)
    print(f"{len(rows)} sessions; {n_with_bin} have >=1 candidate BIN; wrote {OUT_CSV}")
    for row in rows:
        print(f"{row['session_id']:>12} {row['date']} {row['drone']:<12} {row['storage']:<11} "
              f"img={row['n_images']:<5} hdf={row['n_hdf']:<5} bins={row['n_bin']}")


if __name__ == "__main__":
    main()
