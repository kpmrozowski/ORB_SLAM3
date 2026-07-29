#!/usr/bin/env python3
"""Phase B runnability scan for the historical keypoints_summary.csv sessions.

For every session row of eval_out/hist_inventory.csv:
  1. resolve the actual FLIGHT that produced the frames
     - plain dir / zip sessions -> themselves
     - replay_logs sessions     -> the ORIGINAL flight embedded in the parent dir name,
                                   which also pins the exact BIN number (suffix _000000NN)
     - 2026-07-10 golem27 rows  -> the vio/20260710 high-res .temp session (nora zips
                                   carry keypoints only, no frames)
  2. read the session hdf chunks -> frame count + wall-clock span (SensorTimestamp)
  3. scan candidate BINs (pymavlink) -> RISI presence, GPS[1] fix within RADIUS_M of
     TARGET (50.41020638, 29.93702530), GPS-unix span covering the session window
Caches every BIN scan in eval_out/binscan/ so re-runs are cheap.
Writes eval_out/hist_scan.csv with one row per resolved flight and a verdict.
"""
import csv
import glob
import io
import json
import math
import os
import re
import sys
import zipfile

import h5py
import numpy as np
from pymavlink import mavutil

ROOT = "/home/kmro/praca/dev/orbslam3-eval"
FLIGHTS_ROOT = "/media/kmro/datasets/dops/flights"
VIO_ROOT = "/media/kmro/datasets/dops/vio"
INVENTORY_CSV = os.path.join(ROOT, "eval_out", "hist_inventory.csv")
OUT_CSV = os.path.join(ROOT, "eval_out", "hist_scan.csv")
BINSCAN_DIR = os.path.join(ROOT, "eval_out", "binscan")

TARGET_LAT, TARGET_LON = 50.41020638, 29.93702530
RADIUS_M = 1000.0
GPS_EPOCH_UNIX = 315964800  # 1980-01-06, GPS week epoch
LEAP_SECONDS = 18
WINDOW_MARGIN_S = 120.0


def haversine_m(lat_a, lon_a, lat_b, lon_b):
    radius_earth = 6371000.0
    phi_a, phi_b = math.radians(lat_a), math.radians(lat_b)
    delta_phi = math.radians(lat_b - lat_a)
    delta_lambda = math.radians(lon_b - lon_a)
    half_chord = (math.sin(delta_phi / 2.0) ** 2
                  + math.cos(phi_a) * math.cos(phi_b) * math.sin(delta_lambda / 2.0) ** 2)
    return 2.0 * radius_earth * math.asin(math.sqrt(half_chord))


def gps_week_ms_to_unix(gwk, gms):
    return GPS_EPOCH_UNIX + gwk * 604800 + gms / 1000.0 - LEAP_SECONDS


# --------------------------------------------------------------------- flight resolution
def find_bins_case_insensitive(base, stem=None):
    """All ArduPilot logs under base ('.BIN' any case); stem filters by basename."""
    hits = []
    if not os.path.isdir(base):
        return hits
    for dirpath, _dirnames, filenames in os.walk(base):
        for filename in filenames:
            if filename.lower().endswith(".bin") and (stem is None or filename[:-4] == stem):
                hits.append(os.path.join(dirpath, filename))
    return sorted(hits)


def resolve_flight(row):
    """Map an inventory row to the flight that actually produced frames.

    Returns dict(flight_key, session_path, storage, date, drone, forced_bin,
    image_source, note) or None when unresolvable."""
    session_dir = row["session_dir"]
    source_is_replay = "/replay_logs/" in row["session_path"]
    if source_is_replay:
        match = re.search(r"/replay_logs/((\d+)_([a-z0-9-]+)_"
                          r"(\d{4}-\d{2}-\d{2}T\d{2}-\d{2}-\d{2}))_([^/]+)/",
                          row["session_path"] + "/")
        if not match:
            return dict(flight_key="replay:" + session_dir, error="unparseable replay path")
        orig_session, _fid, drone, stamp, bin_number = match.groups()
        date_token = stamp.split("T")[0]
        base = os.path.join(FLIGHTS_ROOT, date_token, drone)
        candidates = (glob.glob(os.path.join(base, "nora", "eval", orig_session))
                      + glob.glob(os.path.join(base, "nora", orig_session))
                      + glob.glob(os.path.join(base, "nora", orig_session + ".temp"))
                      + glob.glob(os.path.join(base, "nora", "eval", orig_session + ".temp")))
        session_path = next((path for path in candidates if os.path.isdir(path)), None)
        zip_candidates = (glob.glob(os.path.join(base, "nora", orig_session + "*.zip"))
                          + glob.glob(os.path.join(base, "nora", "eval", orig_session + "*.zip")))
        forced_bin = None
        bin_hits = find_bins_case_insensitive(base, stem=bin_number)
        if bin_hits:
            forced_bin = sorted(bin_hits, key=lambda path: ("archive" in path, path))[0]
        if session_path:
            return dict(flight_key=orig_session, session_path=session_path, storage="dir",
                        date=date_token, drone=drone, forced_bin=forced_bin,
                        image_source="orig_dir", note=f"replay of {orig_session}")
        if zip_candidates:
            return dict(flight_key=orig_session, session_path=zip_candidates[0], storage="zip",
                        date=date_token, drone=drone, forced_bin=forced_bin,
                        image_source="orig_zip", note=f"replay of {orig_session}")
        return dict(flight_key=orig_session, error="original session dir not found",
                    date=date_token, drone=drone, forced_bin=forced_bin)

    if row["date"] == "2026-07-10" and row["drone"] == "golem27":
        vio_dir = os.path.join(VIO_ROOT, "20260710", session_dir + ".temp")
        if os.path.isdir(os.path.join(vio_dir, "high_res_images")):
            return dict(flight_key=session_dir, session_path=vio_dir, storage="dir",
                        date=row["date"], drone=row["drone"], forced_bin=None,
                        image_source="vio_high_res", note="nora zip has no frames")
        return dict(flight_key=session_dir, error="no frames (zip) and no vio session")

    if row["storage"] == "dir":
        return dict(flight_key=session_dir, session_path=row["session_path"], storage="dir",
                    date=row["date"], drone=row["drone"], forced_bin=None,
                    image_source="session_dir", note="")
    if row["storage"] == "zip":
        return dict(flight_key=session_dir, session_path=row["session_path"], storage="zip",
                    date=row["date"], drone=row["drone"], forced_bin=None,
                    image_source="session_zip", note="")
    return dict(flight_key=session_dir, error=f"storage {row['storage']}")


# --------------------------------------------------------------------- session hdf window
def iter_hdf_chunks(session_path, storage):
    """Yield (name, h5py.File) for every hdf chunk of a session (dir or zip)."""
    if storage == "dir":
        hdf_dir = os.path.join(session_path, "hdf_logs")
        if not os.path.isdir(hdf_dir):
            return
        for name in sorted(os.listdir(hdf_dir), key=lambda item: int(item.split(".")[0])
                           if item.split(".")[0].isdigit() else 1 << 30):
            if name.endswith(".hdf"):
                try:
                    with h5py.File(os.path.join(hdf_dir, name), "r") as handle:
                        yield name, handle
                except OSError:
                    continue
    else:
        with zipfile.ZipFile(session_path) as archive:
            members = sorted((name for name in archive.namelist()
                              if re.fullmatch(r"(?:.*/)?hdf_logs/\d+\.hdf", name)),
                             key=lambda item: int(re.search(r"(\d+)\.hdf", item).group(1)))
            for name in members:
                try:
                    payload = archive.read(name)
                    with h5py.File(io.BytesIO(payload), "r") as handle:
                        yield name, handle
                except (OSError, zipfile.BadZipFile):
                    continue


def session_window(session_path, storage):
    """Frame stamps + wall-clock span from the hdf chunks' keypoints datasets.

    Returns dict(n_frames, first_unix_s, last_unix_s, session_lat, session_lon)."""
    frame_stamp_by_index = {}
    gps_points = []
    for _name, handle in iter_hdf_chunks(session_path, storage):
        try:
            keypoint_rows = handle["localization_data/keypoints"][()]
        except KeyError:
            keypoint_rows = np.empty((0,))
        for record in keypoint_rows:
            source_file = record["SourceFile"]
            source_name = source_file.decode() if isinstance(source_file, bytes) else str(source_file)
            match = re.search(r"(\d+)\.(png|jpg)$", source_name)
            if match:
                frame_stamp_by_index[int(match.group(1))] = float(record["SensorTimestamp"])
        try:
            gps2_rows = handle["mavlink_data/gps2_raw"][()]
            for record in gps2_rows[:: max(1, len(gps2_rows) // 4)]:
                lat, lon = float(record["lat [Degrees]"]), float(record["lon [Degrees]"])
                if abs(lat) > 0.1:
                    gps_points.append((lat, lon))
        except KeyError:
            pass
    if not frame_stamp_by_index:
        return dict(n_frames=0)
    stamps = np.array(sorted(frame_stamp_by_index.values()))
    session_lat = float(np.median([point[0] for point in gps_points])) if gps_points else float("nan")
    session_lon = float(np.median([point[1] for point in gps_points])) if gps_points else float("nan")
    return dict(n_frames=len(frame_stamp_by_index),
                first_unix_s=float(stamps[0]), last_unix_s=float(stamps[-1]),
                session_lat=session_lat, session_lon=session_lon)


# --------------------------------------------------------------------- BIN scanning
def scan_bin(bin_path, full=True, probe_gps_limit=400):
    """Stream-scan a BIN. Cached by absolute path + size.

    Records: RISI count, GPS[1] stats (count, min distance to TARGET, unix span),
    BARO/RFND presence. probe mode stops after probe_gps_limit GPS rows."""
    cache_key = re.sub(r"[^A-Za-z0-9]", "_", bin_path) + f"_{os.path.getsize(bin_path)}"
    cache_path = os.path.join(BINSCAN_DIR, cache_key + (".full.json" if full else ".probe.json"))
    full_cache = os.path.join(BINSCAN_DIR, cache_key + ".full.json")
    if os.path.isfile(full_cache):
        with open(full_cache) as handle:
            return json.load(handle)
    if not full and os.path.isfile(cache_path):
        with open(cache_path) as handle:
            return json.load(handle)

    connection = mavutil.mavlink_connection(bin_path, dialect="ardupilotmega")
    counts = {"RISI": 0, "GPS": 0, "BARO": 0, "RFND": 0}
    gps1_first_unix, gps1_last_unix = None, None
    gps1_count, gps1_fix_count = 0, 0
    min_distance_m = float("inf")
    gps_rows_seen = 0
    while True:
        try:
            message = connection.recv_match(type=["RISI", "GPS", "BARO", "RFND"], blocking=False)
        except Exception:
            break  # corrupt tail (truncated log) - keep whatever was readable
        if message is None:
            break
        message_type = message.get_type()
        counts[message_type] += 1
        if message_type == "GPS":
            gps_rows_seen += 1
            if int(message.I) == 1:
                gps1_count += 1
                unix_time = gps_week_ms_to_unix(int(message.GWk), int(message.GMS))
                if int(message.GWk) > 0:
                    gps1_first_unix = unix_time if gps1_first_unix is None else gps1_first_unix
                    gps1_last_unix = unix_time
                if int(message.Status) >= 3:
                    gps1_fix_count += 1
                    distance = haversine_m(float(message.Lat), float(message.Lng),
                                           TARGET_LAT, TARGET_LON)
                    min_distance_m = min(min_distance_m, distance)
            if not full and gps_rows_seen >= probe_gps_limit:
                break
    result = dict(bin=bin_path, full=full,
                  n_risi=counts["RISI"], n_gps=counts["GPS"], n_baro=counts["BARO"],
                  n_rfnd=counts["RFND"], gps1_count=gps1_count, gps1_fix_count=gps1_fix_count,
                  gps1_min_dist_m=None if math.isinf(min_distance_m) else round(min_distance_m, 1),
                  gps1_first_unix=gps1_first_unix, gps1_last_unix=gps1_last_unix)
    os.makedirs(BINSCAN_DIR, exist_ok=True)
    with open(cache_path, "w") as handle:
        json.dump(result, handle)
    return result


def scan_bin_isolated(bin_path, full):
    """Run scan_bin in a subprocess: pymavlink's C dfindexer hard-exits on corrupt FMT
    tables, which would kill the whole sweep. Results flow through the JSON cache."""
    cache_key = re.sub(r"[^A-Za-z0-9]", "_", bin_path) + f"_{os.path.getsize(bin_path)}"
    full_cache = os.path.join(BINSCAN_DIR, cache_key + ".full.json")
    probe_cache = os.path.join(BINSCAN_DIR, cache_key + ".probe.json")
    wanted = full_cache if full else (full_cache if os.path.isfile(full_cache) else probe_cache)
    if not os.path.isfile(wanted):
        import subprocess
        subprocess.run([sys.executable, os.path.abspath(__file__), "--scan-one", bin_path,
                        "full" if full else "probe"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        wanted = full_cache if (full or os.path.isfile(full_cache)) else probe_cache
    if not os.path.isfile(wanted):
        raise RuntimeError("BIN unreadable (indexer died)")
    with open(wanted) as handle:
        return json.load(handle)


def pick_and_scan_bins(candidate_bins, forced_bin, window):
    """Probe candidates cheaply, then full-scan the ones plausibly covering the window."""
    if forced_bin:
        candidate_bins = [forced_bin]
    session_start = window.get("first_unix_s")
    survivors = []
    for bin_path in candidate_bins:
        if len(candidate_bins) == 1 or session_start is None:
            survivors.append(bin_path)
            continue
        try:
            probe = scan_bin_isolated(bin_path, full=False)
        except Exception as error:
            print(f"  probe failed on {bin_path}: {error}")
            continue
        probe_start = probe.get("gps1_first_unix")
        if probe_start is None:
            survivors.append(bin_path)  # cannot pre-filter without a GPS clock
            continue
        if probe_start - WINDOW_MARGIN_S <= session_start <= probe_start + 4 * 3600:
            survivors.append(bin_path)
    scans = []
    for bin_path in survivors:
        try:
            scan = scan_bin_isolated(bin_path, full=True)
        except Exception as error:
            print(f"  scan failed on {bin_path}: {error}")
            continue
        if session_start is not None and scan["gps1_first_unix"] is not None:
            covers = (scan["gps1_first_unix"] - WINDOW_MARGIN_S <= session_start
                      and scan["gps1_last_unix"] + WINDOW_MARGIN_S >= window["last_unix_s"])
            scan["covers_session"] = bool(covers)
        else:
            scan["covers_session"] = None
        scans.append(scan)
    scans.sort(key=lambda scan: (scan["covers_session"] is not True, -scan["n_risi"]))
    return scans


# --------------------------------------------------------------------- main
def main():
    if len(sys.argv) >= 4 and sys.argv[1] == "--scan-one":
        scan_bin(sys.argv[2], full=(sys.argv[3] == "full"))
        return
    only_key = sys.argv[1] if len(sys.argv) > 1 else None
    with open(INVENTORY_CSV, newline="") as handle:
        inventory = list(csv.DictReader(handle))

    resolved_by_key = {}
    for row in inventory:
        resolved = resolve_flight(row)
        if resolved is None:
            continue
        key = resolved["flight_key"]
        entry = resolved_by_key.setdefault(key, dict(resolved=resolved, session_ids=[],
                                                     bin_paths=row["bin_paths"]))
        entry["session_ids"].append(row["session_id"] + ":" + row["session_dir"])
        if row["bin_paths"] and not entry["bin_paths"]:
            entry["bin_paths"] = row["bin_paths"]

    out_rows = []
    for key, entry in resolved_by_key.items():
        if only_key and only_key not in key:
            continue
        resolved = entry["resolved"]
        record = dict(flight_key=key, session_ids=";".join(entry["session_ids"]),
                      date=resolved.get("date", "?"), drone=resolved.get("drone", "?"),
                      image_source=resolved.get("image_source", ""),
                      note=resolved.get("note", ""))
        if "error" in resolved:
            record.update(verdict="NOT_RUNNABLE", reason=resolved["error"])
            out_rows.append(record)
            print(f"{key}: NOT_RUNNABLE ({resolved['error']})")
            continue
        window = session_window(resolved["session_path"], resolved["storage"])
        record.update(session_path=resolved["session_path"], n_frames=window.get("n_frames", 0),
                      first_unix_s=window.get("first_unix_s"), last_unix_s=window.get("last_unix_s"),
                      session_lat=window.get("session_lat"), session_lon=window.get("session_lon"))
        if window.get("n_frames", 0) == 0:
            record.update(verdict="NOT_RUNNABLE", reason="no frame stamps in hdf_logs")
            out_rows.append(record)
            print(f"{key}: NOT_RUNNABLE (no frame stamps)")
            continue
        drone_dir = os.path.join(FLIGHTS_ROOT, resolved["date"], resolved["drone"])
        candidate_bins = find_bins_case_insensitive(drone_dir)
        scans = pick_and_scan_bins(candidate_bins, resolved.get("forced_bin"), window)
        best = next((scan for scan in scans
                     if scan["n_risi"] > 0 and scan.get("covers_session") is not False), None)
        if best is None:
            reason = "no BIN candidates" if not (candidate_bins or resolved.get("forced_bin")) \
                else "no covering BIN with RISI"
            record.update(verdict="NOT_RUNNABLE", reason=reason, n_bins_scanned=len(scans))
            out_rows.append(record)
            print(f"{key}: NOT_RUNNABLE ({reason})")
            continue
        near = best["gps1_min_dist_m"] is not None and best["gps1_min_dist_m"] <= RADIUS_M
        record.update(bin=best["bin"], n_risi=best["n_risi"], n_baro=best["n_baro"],
                      n_rfnd=best["n_rfnd"], gps1_fix_count=best["gps1_fix_count"],
                      gps1_min_dist_m=best["gps1_min_dist_m"],
                      covers_session=best["covers_session"],
                      verdict="RUNNABLE" if near else "NOT_RUNNABLE",
                      reason="" if near else f"GPS[1] min dist {best['gps1_min_dist_m']} m > {RADIUS_M:.0f}")
        out_rows.append(record)
        print(f"{key}: {record['verdict']} bin={os.path.basename(best['bin'])} "
              f"risi={best['n_risi']} gps1min={best['gps1_min_dist_m']} covers={best['covers_session']}")

    fieldnames = sorted({name for row in out_rows for name in row})
    with open(OUT_CSV, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(out_rows)
    n_runnable = sum(1 for row in out_rows if row["verdict"] == "RUNNABLE")
    print(f"\n{len(out_rows)} flights, {n_runnable} RUNNABLE -> {OUT_CSV}")


if __name__ == "__main__":
    main()
