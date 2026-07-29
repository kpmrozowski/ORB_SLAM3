#!/usr/bin/env python3
"""Convert a RUNNABLE historical flight (hist_scan.csv row) to the common EuRoC eval
layout under /home/kmro/praca/dev/orbslam3-eval/datasets-nora/<flight_key>/.

Per flight:
  * frame stamps : hdf_logs keypoints SensorTimestamp (companion Unix s) keyed by image
                   index parsed from SourceFile; images symlinked (dir) / extracted (zip)
                   as <rebased_ns>.png. vio_high_res flights use ns-named high_res_images
                   downscaled 1640x1232 -> 800x600 (INTER_AREA), as for 538/542.
  * clock bridge : per-flight fit companion_ns = slope * fc_boot_ns + K from the hdf
                   mavlink timesync (tc1 = FC boot ns, ts1 = companion ns); fallback to
                   system_time (companion s, time_since_boot ms). Reuses
                   nora_bin.BridgeParams so RISI TimeUS maps exactly like 538/542.
  * IMU          : BIN RISI (preintegrated replay IMU) -> gyro/accel via
                   nora_bin.risi_to_imu, stamped through the bridge, shifted by
                   -timeshift_cam_imu (kalibr) from the session calibration.yaml.
  * sidecars     : ref_baro.csv, ref_rfnd.csv, ref_att.csv, ref_gps1.csv (rebased ns).
  * config       : configs/hist/<flight_key>.yaml - KB4 intrinsics transformed
                   800x600 -> crop [cx,cy] -> x0.5 -> 300x225 (the NORA downsampling),
                   T_b_c1 = inv(T_cam_imu), nFeatures 10000, FAST 5/1.

BIN parse is cached in eval_out/bincache/<flight_key>/ (parse_bin schema + ATT).
"""
import argparse
import csv
import io
import json
import os
import re
import sys
import zipfile

import numpy as np
import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, os.path.join(ROOT, "nora_20260709", "lib"))
sys.path.insert(0, SCRIPT_DIR)
import nora_bin  # noqa: E402
from hist_scan import FLIGHTS_ROOT, find_bins_case_insensitive, iter_hdf_chunks  # noqa: E402

SCAN_CSV = os.path.join(ROOT, "eval_out", "hist_scan.csv")
BINCACHE_ROOT = os.path.join(ROOT, "eval_out", "bincache")
DATASETS_ROOT = os.path.join(ROOT, "datasets-nora")
CONFIG_DIR = os.path.join(ROOT, "configs", "hist")
IMU_MARGIN_NS = 2_000_000_000


# --------------------------------------------------------------------- session artefacts
def read_session_yaml(session_path, storage, member):
    """Load a yaml file (calibration.yaml / hydra/config.yaml) from a dir or zip session."""
    if storage == "dir":
        full = os.path.join(session_path, member)
        if not os.path.isfile(full):
            return None
        with open(full) as handle:
            return yaml.safe_load(handle)
    with zipfile.ZipFile(session_path) as archive:
        names = [name for name in archive.namelist() if name.endswith(member)]
        if not names:
            return None
        return yaml.safe_load(io.BytesIO(archive.read(sorted(names, key=len)[0])))


def find_crop(node):
    """Locate the camera crop [x_margin, y_margin] + resolution in the hydra config tree.
    Some fleets (epos3) set `crop: null` - treated as no crop."""
    if isinstance(node, dict):
        if "crop" in node and "resolution" in node and node["resolution"] is not None:
            crop = node["crop"] if node["crop"] is not None else [0, 0]
            return list(node["resolution"]), list(crop)
        for value in node.values():
            found = find_crop(value)
            if found:
                return found
    elif isinstance(node, list):
        for value in node:
            found = find_crop(value)
            if found:
                return found
    return None


def collect_streams(session_path, storage):
    """One pass over the hdf chunks: frame stamps by image index + clock-bridge pairs
    (+ gps2 companion<->GPS-unix pairs for the intermediary-bridge fallback)."""
    frame_stamp_by_index = {}
    timesync_pairs, system_time_pairs, gps2_pairs = [], [], []
    for _name, handle in iter_hdf_chunks(session_path, storage):
        try:
            for record in handle["localization_data/keypoints"][()]:
                source_file = record["SourceFile"]
                source_name = source_file.decode() if isinstance(source_file, bytes) else str(source_file)
                match = re.search(r"(\d+)\.(png|jpg)$", source_name)
                if match:
                    frame_stamp_by_index.setdefault(int(match.group(1)),
                                                    float(record["SensorTimestamp"]))
        except KeyError:
            pass
        try:
            for record in handle["mavlink_data/timesync"][()]:
                timesync_pairs.append((float(record["tc1 [Nanoseconds]"]),
                                       float(record["ts1 [Nanoseconds]"])))
        except KeyError:
            pass
        try:
            for record in handle["mavlink_data/system_time"][()]:
                system_time_pairs.append((float(record["time_since_boot [Milliseconds]"]) * 1e6,
                                          float(record["timestamp [Seconds]"]) * 1e9))
        except KeyError:
            pass
        try:
            for record in handle["mavlink_data/gps2_raw"][()]:
                if float(record["time_usec [Microseconds]"]) > 0:
                    gps2_pairs.append((float(record["time_usec [Microseconds]"]) * 1e3,
                                       float(record["timestamp [Seconds]"]) * 1e9))
        except KeyError:
            pass
    return (frame_stamp_by_index, np.array(timesync_pairs), np.array(system_time_pairs),
            np.array(gps2_pairs))


def fit_bridge(timesync_pairs, system_time_pairs, window_ns=None):
    """Fit companion_ns = slope * fc_boot_ns + K. Returns (BridgeParams, source, residual_ms)."""
    for pairs, source in ((timesync_pairs, "timesync"), (system_time_pairs, "system_time")):
        if len(pairs) < 8:
            continue
        # TIMESYNC direction varies per row (request vs response), so tc1/ts1 roles can
        # swap: classify by magnitude instead (FC boot ns ~1e12-1e13 vs Unix ns ~1.7e18).
        low = np.minimum(pairs[:, 0], pairs[:, 1])
        high = np.maximum(pairs[:, 0], pairs[:, 1])
        keep = (low > 0) & (low < 1e16) & (high > 1e17)
        # Same-day sessions can carry pairs from OTHER FC boots (parallel line segments
        # hours apart -> garbage single-line fit): keep only pairs inside the session.
        if window_ns is not None:
            keep &= (high >= window_ns[0] - 120e9) & (high <= window_ns[1] + 120e9)
        boot_ns, companion_ns = low[keep], high[keep]
        if len(boot_ns) < 8:
            continue
        # An FC reboot mid-session yields SEVERAL (boot, companion) line segments with
        # offsets minutes apart (one per boot). Fit only the dominant offset cluster;
        # frames from the other boot fall outside the IMU window and are dropped.
        offset_bucket = np.round((companion_ns - boot_ns) / 30e9)
        bucket_values, bucket_counts = np.unique(offset_bucket, return_counts=True)
        dominant = bucket_values[np.argmax(bucket_counts)]
        cluster = np.abs(offset_bucket - dominant) <= 1
        if cluster.sum() < len(boot_ns):
            print(f"  bridge[{source}]: {len(boot_ns) - cluster.sum()} pairs from other "
                  f"FC boots discarded (mid-session reboot)")
        boot_ns, companion_ns = boot_ns[cluster], companion_ns[cluster]
        if len(boot_ns) < 8:
            continue
        boot_mean, companion_mean = boot_ns.mean(), companion_ns.mean()
        slope = (np.sum((boot_ns - boot_mean) * (companion_ns - companion_mean))
                 / np.sum((boot_ns - boot_mean) ** 2))
        offset_ns = companion_mean - slope * boot_mean
        residual_ns = companion_ns - (slope * boot_ns + offset_ns)
        median_abs_ms = float(np.median(np.abs(residual_ns))) / 1e6
        # One robust re-fit without outliers (queueing delays make positive tails).
        inliers = np.abs(residual_ns) < max(5.0 * np.median(np.abs(residual_ns)), 1e6)
        if inliers.sum() >= 8:
            boot_in, companion_in = boot_ns[inliers], companion_ns[inliers]
            boot_mean, companion_mean = boot_in.mean(), companion_in.mean()
            slope = (np.sum((boot_in - boot_mean) * (companion_in - companion_mean))
                     / np.sum((boot_in - boot_mean) ** 2))
            offset_ns = companion_mean - slope * boot_mean
            residual_ns = companion_in - (slope * boot_in + offset_ns)
            median_abs_ms = float(np.median(np.abs(residual_ns))) / 1e6
        if not 0.9 < slope < 1.1:
            print(f"  bridge[{source}] rejected: non-physical slope {slope:.6f}")
            continue
        params = nora_bin.BridgeParams(slope=float(slope), k_seconds=float(offset_ns) / 1e9)
        return params, source, median_abs_ms
    return None, None, None


def fit_bridge_via_gps(gps2_pairs, gps_arrays):
    """Fallback bridge when timesync/system_time are unusable (old sessions): compose
    session-side companion<->GPS-unix (hdf gps2 time_usec) with BIN-side
    TimeUS<->GPS-unix (GPS[1] GWk/GMS), eliminating the GPS-unix intermediary."""
    if len(gps2_pairs) < 8:
        raise SystemExit("no usable clock-bridge pairs (timesync/system_time/gps2)")
    from hist_scan import gps_week_ms_to_unix
    keep = (gps_arrays["inst"] == 1) & (gps_arrays["gwk"] > 0)
    if keep.sum() < 8:
        keep = (gps_arrays["inst"] == 0) & (gps_arrays["gwk"] > 0)
    if keep.sum() < 8:
        raise SystemExit("BIN has no GPS rows with a valid week for the gps2 fallback")
    bin_timeus = gps_arrays["timeus"][keep].astype(np.float64)
    bin_unix_ns = np.array([gps_week_ms_to_unix(int(gwk), int(gms)) * 1e9
                            for gwk, gms in zip(gps_arrays["gwk"][keep],
                                                gps_arrays["gms"][keep])])
    slope_a = np.polyfit(bin_timeus - bin_timeus.mean(), bin_unix_ns - bin_unix_ns.mean(), 1)[0]
    offset_a = bin_unix_ns.mean() - slope_a * bin_timeus.mean()  # unix_ns = a*TimeUS + b
    unix_ns, companion_ns = gps2_pairs[:, 0], gps2_pairs[:, 1]
    slope_c = np.polyfit(unix_ns - unix_ns.mean(), companion_ns - companion_ns.mean(), 1)[0]
    offset_c = companion_ns.mean() - slope_c * unix_ns.mean()  # companion_ns = c*unix_ns + d
    residual_ns = companion_ns - (slope_c * unix_ns + offset_c)
    # companion_ns = c*(a*TimeUS + b) + d ; BridgeParams: companion_ns = slope*1000*TimeUS + K
    slope = slope_c * slope_a / 1000.0
    k_seconds = (slope_c * offset_a + offset_c) / 1e9
    if not 0.9 < slope < 1.1:
        raise SystemExit(f"gps2 fallback produced non-physical slope {slope:.6f}")
    params = nora_bin.BridgeParams(slope=float(slope), k_seconds=float(k_seconds))
    return params, "gps2+binGPS", float(np.median(np.abs(residual_ns))) / 1e6


# --------------------------------------------------------------------- BIN cache
def bin_cache_dir(bin_path):
    return os.path.join(BINCACHE_ROOT,
                        re.sub(r"[^A-Za-z0-9]", "_", bin_path)
                        + f"_{os.path.getsize(bin_path)}")


def parse_bin_cached(bin_path, cache_dir):
    """parse_bin.py schema (risi/gps/baro/rfnd/mag npz, tag 'hist') + ATT, cached."""
    marker = os.path.join(cache_dir, "hist_att.npz")
    if os.path.isfile(marker):
        return
    from pymavlink import mavutil
    os.makedirs(cache_dir, exist_ok=True)
    connection = mavutil.mavlink_connection(bin_path, dialect="ardupilotmega")
    risi, gps, baro, rfnd, mag, att = [], [], [], [], [], []
    last_rfrh_timeus = -1
    while True:
        try:
            message = connection.recv_match(
                type=["RFRH", "RISI", "GPS", "BARO", "RFND", "MAG", "ATT"], blocking=False)
        except Exception:
            break
        if message is None:
            break
        message_type = message.get_type()
        if message_type == "RFRH":
            last_rfrh_timeus = int(message.TimeUS)
        elif message_type == "RISI":
            if int(message.I) == 0 and last_rfrh_timeus >= 0:
                risi.append((last_rfrh_timeus, 0.0, int(message.Flags),
                             float(message.DVX), float(message.DVY), float(message.DVZ),
                             float(message.DAX), float(message.DAY), float(message.DAZ),
                             float(message.DVDT), float(message.DADT)))
        elif message_type == "GPS":
            gps.append((int(message.TimeUS), int(message.I), int(message.Status),
                        int(message.GMS), int(message.GWk), int(message.NSats),
                        float(message.Lat), float(message.Lng), float(message.Alt),
                        float(message.Spd), 0.0))
        elif message_type == "BARO":
            baro.append((int(message.TimeUS), int(message.I),
                         float(message.Alt), float(message.Press), float(message.Temp)))
        elif message_type == "RFND":
            rfnd.append((int(message.TimeUS), int(message.Instance),
                         float(message.Dist), int(message.Stat),
                         int(getattr(message, "Quality", -1))))
        elif message_type == "MAG":
            mag.append((int(message.TimeUS), int(message.I),
                        int(message.MagX), int(message.MagY), int(message.MagZ)))
        elif message_type == "ATT":
            att.append((int(message.TimeUS), float(message.Roll), float(message.Pitch),
                        float(message.Yaw)))

    risi_arr, gps_arr = np.asarray(risi, dtype=np.float64), np.asarray(gps, dtype=np.float64)
    baro_arr, rfnd_arr = np.asarray(baro, dtype=np.float64), np.asarray(rfnd, dtype=np.float64)
    mag_arr, att_arr = np.asarray(mag, dtype=np.float64), np.asarray(att, dtype=np.float64)
    np.savez(os.path.join(cache_dir, "hist_risi.npz"),
             timeus=risi_arr[:, 0].astype(np.int64), dfts=risi_arr[:, 1],
             flags=risi_arr[:, 2].astype(np.int64),
             dvx=risi_arr[:, 3], dvy=risi_arr[:, 4], dvz=risi_arr[:, 5],
             dax=risi_arr[:, 6], day=risi_arr[:, 7], daz=risi_arr[:, 8],
             dvdt=risi_arr[:, 9], dadt=risi_arr[:, 10])
    np.savez(os.path.join(cache_dir, "hist_gps.npz"),
             timeus=gps_arr[:, 0].astype(np.int64), inst=gps_arr[:, 1].astype(np.int64),
             status=gps_arr[:, 2].astype(np.int64), gms=gps_arr[:, 3].astype(np.int64),
             gwk=gps_arr[:, 4].astype(np.int64), nsats=gps_arr[:, 5].astype(np.int64),
             lat=gps_arr[:, 6], lng=gps_arr[:, 7], alt=gps_arr[:, 8], spd=gps_arr[:, 9],
             dfts=gps_arr[:, 10])
    np.savez(os.path.join(cache_dir, "hist_baro.npz"),
             timeus=baro_arr[:, 0].astype(np.int64), inst=baro_arr[:, 1].astype(np.int64),
             alt=baro_arr[:, 2], press=baro_arr[:, 3], temp=baro_arr[:, 4])
    np.savez(os.path.join(cache_dir, "hist_rfnd.npz"),
             timeus=rfnd_arr[:, 0].astype(np.int64) if rfnd_arr.size else np.empty(0, np.int64),
             inst=rfnd_arr[:, 1].astype(np.int64) if rfnd_arr.size else np.empty(0, np.int64),
             dist=rfnd_arr[:, 2] if rfnd_arr.size else np.empty(0),
             stat=rfnd_arr[:, 3].astype(np.int64) if rfnd_arr.size else np.empty(0, np.int64),
             quality=rfnd_arr[:, 4].astype(np.int64) if rfnd_arr.size else np.empty(0, np.int64))
    np.savez(os.path.join(cache_dir, "hist_mag.npz"),
             timeus=mag_arr[:, 0].astype(np.int64) if mag_arr.size else np.empty(0, np.int64),
             inst=mag_arr[:, 1].astype(np.int64) if mag_arr.size else np.empty(0, np.int64),
             magx=mag_arr[:, 2] if mag_arr.size else np.empty(0),
             magy=mag_arr[:, 3] if mag_arr.size else np.empty(0),
             magz=mag_arr[:, 4] if mag_arr.size else np.empty(0))
    np.savez(os.path.join(cache_dir, "hist_att.npz"),
             timeus=att_arr[:, 0].astype(np.int64) if att_arr.size else np.empty(0, np.int64),
             roll=att_arr[:, 1] if att_arr.size else np.empty(0),
             pitch=att_arr[:, 2] if att_arr.size else np.empty(0),
             yaw=att_arr[:, 3] if att_arr.size else np.empty(0))
    print(f"  BIN cached: risi={len(risi)} gps={len(gps)} baro={len(baro)} "
          f"rfnd={len(rfnd)} mag={len(mag)} att={len(att)}")


# --------------------------------------------------------------------- writers
def write_imu_csv(out_csv, stamps_ns, samples, epoch_ns):
    with open(out_csv, "w", newline="") as handle:
        handle.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                     "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        writer = csv.writer(handle)
        for stamp, sample in zip(stamps_ns, samples):
            writer.writerow([int(stamp) - epoch_ns] + [f"{value:.9f}" for value in sample])


def write_reference(out_csv, header, stamps_ns, columns, epoch_ns):
    with open(out_csv, "w", newline="") as handle:
        handle.write(header + "\n")
        for row_index in range(len(stamps_ns)):
            values = ",".join(f"{column[row_index]:.6f}" for column in columns)
            handle.write(f"{int(stamps_ns[row_index]) - epoch_ns},{values}\n")


def emit_frames(scan_row, frame_stamps_ns, frame_indices, out_dir, epoch_ns):
    """Symlink (dir) or extract (zip) the 300x225 session images as <rebased>.png.
    vio_high_res flights instead downscale 1640x1232 -> 800x600 (INTER_AREA) copies."""
    cam_dir = os.path.join(out_dir, "mav0", "cam0", "data")
    os.makedirs(cam_dir, exist_ok=True)
    session_path, storage = scan_row["session_path"], scan_row_storage(scan_row)
    written = 0
    if scan_row["image_source"] == "vio_high_res":
        import cv2
        image_dir = vio_image_dir(session_path)
        for stamp_ns, frame_index in zip(frame_stamps_ns, frame_indices):
            image = cv2.imread(os.path.join(image_dir, f"{frame_index}.png"),
                               cv2.IMREAD_GRAYSCALE)
            if image is None:
                continue
            downscaled = cv2.resize(image, (800, 600), interpolation=cv2.INTER_AREA)
            cv2.imwrite(os.path.join(cam_dir, f"{stamp_ns - epoch_ns}.png"), downscaled)
            written += 1
    elif storage == "dir":
        image_dir = os.path.join(session_path, "images")
        for stamp_ns, frame_index in zip(frame_stamps_ns, frame_indices):
            source = os.path.join(image_dir, f"{frame_index}.png")
            target = os.path.join(cam_dir, f"{stamp_ns - epoch_ns}.png")
            if not os.path.isfile(source) or os.path.getsize(source) == 0:
                continue
            if os.path.lexists(target):
                os.remove(target)
            os.symlink(source, target)
            written += 1
    else:
        with zipfile.ZipFile(session_path) as archive:
            member_by_index = {}
            for name in archive.namelist():
                match = re.fullmatch(r"(?:.*/)?images/(\d+)\.png", name)
                if match:
                    member_by_index[int(match.group(1))] = name
            for stamp_ns, frame_index in zip(frame_stamps_ns, frame_indices):
                member = member_by_index.get(frame_index)
                if member is None:
                    continue
                payload = archive.read(member)
                if not payload:
                    continue
                with open(os.path.join(cam_dir, f"{stamp_ns - epoch_ns}.png"), "wb") as handle:
                    handle.write(payload)
                written += 1
    return written


def scan_row_storage(scan_row):
    return "zip" if scan_row["session_path"].endswith(".zip") else "dir"


def vio_image_dir(session_path):
    for name in ("high_res_images", "high_res_images_png"):
        candidate = os.path.join(session_path, name)
        if os.path.isdir(candidate) and any(entry.endswith(".png")
                                            for entry in os.listdir(candidate)[:50]):
            return candidate
    raise SystemExit("no ns-named PNG dir in vio session")


def resolve_image_geometry(stored_size, calibration, hydra_camera):
    """How the stored frames were derived from the calibration resolution.

    Returns (crop_x, crop_y, scale, profile). Sessions are heterogeneous: some store
    native 800x600 frames, others the NORA crop [cx,cy] -> resize-to-min-225 pipeline."""
    stored_width, stored_height = stored_size
    calib_width, calib_height = (int(value) for value in calibration["resolution"])
    if (stored_width, stored_height) == (calib_width, calib_height):
        return 0, 0, 1.0, "native"
    if hydra_camera is not None:
        resolution, crop = hydra_camera
        crop_x, crop_y = crop
        cropped_width = resolution[0] - 2 * crop_x
        cropped_height = resolution[1] - 2 * crop_y
        if abs(stored_width / stored_height - cropped_width / cropped_height) < 0.02:
            return crop_x, crop_y, stored_height / cropped_height, "cropped_scaled"
    if abs(stored_width / stored_height - calib_width / calib_height) < 0.02:
        return 0, 0, stored_width / calib_width, "scaled"
    raise SystemExit(f"cannot relate stored {stored_width}x{stored_height} to calibration "
                     f"{calib_width}x{calib_height} with crop {hydra_camera}")


def write_config(flight_key, calibration, hydra_camera, fps, imu_hz, stored_size):
    """Emit configs/hist/<flight_key>.yaml with intrinsics matched to the STORED frame
    geometry (probed from an emitted image, see resolve_image_geometry).

    Low-res (<=300 px tall) extractor tuning (measured on 90_golem23, batch hist_var1):
    stock 1.2/8 pyramid at native 300px pools only ~7.4k FAST corners at threshold 1;
    scaleFactor 1.1 + nLevels 12 + a 1.5x config upscale (Camera.newWidth, ORB-SLAM3
    rescales K itself) saturates >=10k detections on ~99% of frames; nFeatures 11000
    leaves headroom over the octree cap. Full-res sessions use the proven 538/542
    profile (nf5000, FAST 15/5, stock pyramid)."""
    fx, fy, cx, cy = calibration["intrinsics"]
    crop_x, crop_y, scale, profile = resolve_image_geometry(stored_size, calibration,
                                                            hydra_camera)
    out_width, out_height = stored_size
    dense = out_height <= 300
    n_features = 11000 if dense else 5000
    fast_ini, fast_min = (5, 1) if dense else (15, 5)
    scale_factor, n_levels = ("1.1", "12") if dense else ("1.2", "8")
    upscale_lines = ([f"Camera.newWidth: {int(round(out_width * 1.5))}",
                      f"Camera.newHeight: {int(round(out_height * 1.5))}", ""]
                     if dense else [])
    k1, k2, k3, k4 = calibration["distortion_coeffs"]
    T_cam_imu = np.array(calibration["T_cam_imu"], dtype=np.float64)
    T_imu_cam = np.linalg.inv(T_cam_imu)
    os.makedirs(CONFIG_DIR, exist_ok=True)
    config_path = os.path.join(CONFIG_DIR, f"{flight_key}.yaml")
    calib_width, calib_height = (int(value) for value in calibration["resolution"])
    body = "\n".join([
        "%YAML:1.0",
        f"# {flight_key} historical NORA flight - MONO-INERTIAL @ {out_width}x{out_height}",
        f"# intrinsics: calib {calib_width}x{calib_height} -> crop [{crop_x},{crop_y}] "
        f"-> x{scale:.3f} (stored-image geometry: {profile})",
        "File.version: \"1.0\"", "",
        "Camera.type: \"KannalaBrandt8\"", "",
        f"Camera1.fx: {fx * scale:.6f}",
        f"Camera1.fy: {fy * scale:.6f}",
        f"Camera1.cx: {(cx - crop_x) * scale:.6f}",
        f"Camera1.cy: {(cy - crop_y) * scale:.6f}", "",
        f"Camera1.k1: {k1:.10f}",
        f"Camera1.k2: {k2:.10f}",
        f"Camera1.k3: {k3:.10f}",
        f"Camera1.k4: {k4:.10f}", "",
        f"Camera.width: {out_width}",
        f"Camera.height: {out_height}",
        f"Camera.fps: {max(1, int(round(fps)))}",
        "Camera.RGB: 1", ""]
        + upscale_lines
        + [
        "IMU.T_b_c1: !!opencv-matrix",
        "   rows: 4",
        "   cols: 4",
        "   dt: f",
        "   data: [ " + ",\n           ".join(
            ", ".join(f"{T_imu_cam[row, col]:.16f}" for col in range(4)) for row in range(4)) + "]",
        "",
        "IMU.InsertKFsWhenLost: 1",
        "IMU.NoiseGyro: 5.0e-4",
        "IMU.NoiseAcc: 1.3e-2",
        "IMU.GyroWalk: 4.0e-6",
        "IMU.AccWalk: 9.0e-4",
        f"IMU.Frequency: {imu_hz:.2f}", "",
        f"ORBextractor.nFeatures: {n_features}",
        f"ORBextractor.scaleFactor: {scale_factor}",
        f"ORBextractor.nLevels: {n_levels}",
        f"ORBextractor.iniThFAST: {fast_ini}",
        f"ORBextractor.minThFAST: {fast_min}", "",
        "Viewer.KeyFrameSize: 0.05",
        "Viewer.KeyFrameLineWidth: 1.0",
        "Viewer.GraphLineWidth: 0.9",
        "Viewer.PointSize: 2.0",
        "Viewer.CameraSize: 0.08",
        "Viewer.CameraLineWidth: 3.0",
        "Viewer.ViewpointX: 0.0",
        "Viewer.ViewpointY: -0.7",
        "Viewer.ViewpointZ: -3.5",
        "Viewer.ViewpointF: 500.0", "",
    ])
    with open(config_path, "w") as handle:
        handle.write(body)
    return config_path, out_width, out_height


# --------------------------------------------------------------------- conversion
def convert_flight(scan_row):
    flight_key = scan_row["flight_key"]
    storage = scan_row_storage(scan_row)
    session_path = scan_row["session_path"]
    out_dir = os.path.join(DATASETS_ROOT, flight_key)
    print(f"== {flight_key} ({storage}, {scan_row['image_source']}) ==")
    is_vio = scan_row["image_source"] == "vio_high_res"

    (frame_stamp_by_index, timesync_pairs, system_time_pairs,
     gps2_pairs) = collect_streams(session_path, storage)
    if is_vio:
        # 538/542-style session: companion-ns-named 1640x1232 frames; stamps ARE the names.
        # 20260710 sessions keep raw .gray in high_res_images and PNGs in *_png.
        image_dir = vio_image_dir(session_path)
        frame_stamp_by_index = {
            stamp: stamp / 1e9
            for stamp in sorted(
                int(name[:-4]) for name in os.listdir(image_dir)
                if name.endswith(".png") and name[:-4].isdigit()
                and os.path.getsize(os.path.join(image_dir, name)) > 0)}
    if not frame_stamp_by_index:
        raise SystemExit("no frame stamps")
    session_window_ns = (min(frame_stamp_by_index.values()) * 1e9,
                         max(frame_stamp_by_index.values()) * 1e9)
    bridge, bridge_source, residual_ms = fit_bridge(timesync_pairs, system_time_pairs,
                                                    window_ns=session_window_ns)
    if bridge is None:
        fallback_cache = bin_cache_dir(scan_row["bin"])
        parse_bin_cached(scan_row["bin"], fallback_cache)
        fallback_data = nora_bin.load_cached(fallback_cache, "hist")
        bridge, bridge_source, residual_ms = fit_bridge_via_gps(gps2_pairs,
                                                                fallback_data["gps"])
    print(f"  bridge[{bridge_source}]: slope={bridge.slope:.9f} "
          f"K={bridge.k_seconds:.6f}s median|res|={residual_ms:.2f}ms n_ts={len(timesync_pairs)}")

    # The scan's GPS-time BIN pick can be fooled by cross-date log copies in gt/ dirs.
    # The bridge gives the definitive test: the flight's FC-boot TimeUS window must lie
    # inside the BIN's RISI TimeUS range. Fall back to sibling candidates on mismatch.
    frame_lo_ns = int(round(min(frame_stamp_by_index.values()) * 1e9))
    frame_hi_ns = int(round(max(frame_stamp_by_index.values()) * 1e9))
    expected_lo_us = int(nora_bin.companion_ns_to_timeus(frame_lo_ns, bridge))
    expected_hi_us = int(nora_bin.companion_ns_to_timeus(frame_hi_ns, bridge))
    candidates = [scan_row["bin"]] + [
        path for path in find_bins_case_insensitive(
            os.path.join(FLIGHTS_ROOT, scan_row["date"], scan_row["drone"]))
        if path != scan_row["bin"]]
    cached, cache_dir = None, None
    best_partial = None  # (overlap_us, bin_path, data, cache_dir)
    for bin_path in candidates:
        try:
            bin_cache = bin_cache_dir(bin_path)
            parse_bin_cached(bin_path, bin_cache)
            candidate_data = nora_bin.load_cached(bin_cache, "hist")
        except Exception as error:
            print(f"  candidate {os.path.basename(bin_path)} unreadable: {error}")
            continue
        risi_timeus = candidate_data["risi"]["timeus"]
        if risi_timeus.size == 0:
            continue
        margin_us = 2_000_000
        if (risi_timeus.min() <= expected_lo_us + margin_us
                and risi_timeus.max() >= expected_hi_us - margin_us):
            if bin_path != scan_row["bin"]:
                print(f"  BIN re-picked by TimeUS overlap: {os.path.basename(bin_path)}")
            scan_row = dict(scan_row, bin=bin_path)
            cached, cache_dir = candidate_data, bin_cache
            break
        overlap_us = (min(int(risi_timeus.max()), expected_hi_us)
                      - max(int(risi_timeus.min()), expected_lo_us))
        if best_partial is None or overlap_us > best_partial[0]:
            best_partial = (overlap_us, bin_path, candidate_data, bin_cache)
        print(f"  candidate {os.path.basename(bin_path)} rejected: RISI TimeUS "
              f"[{int(risi_timeus.min())},{int(risi_timeus.max())}] vs expected "
              f"[{expected_lo_us},{expected_hi_us}] (overlap {overlap_us / 1e6:.0f}s)")
    if cached is None and best_partial is not None and best_partial[0] >= 120_000_000:
        overlap_us, bin_path, cached, cache_dir = best_partial
        print(f"  accepting PARTIAL coverage {os.path.basename(bin_path)} "
              f"({overlap_us / 1e6:.0f}s of {(expected_hi_us - expected_lo_us) / 1e6:.0f}s; "
              f"frames outside the IMU window are dropped)")
        scan_row = dict(scan_row, bin=bin_path)
    if cached is None:
        raise SystemExit("no candidate BIN overlaps the flight's FC-boot window")

    calibration = read_session_yaml(session_path, storage, "calibration.yaml")
    hydra_config = read_session_yaml(session_path, storage, "hydra/config.yaml")
    hydra_camera = None if is_vio else (find_crop(hydra_config) if hydra_config else None)
    if calibration is None:
        raise SystemExit("missing calibration.yaml")
    timeshift_s = float(calibration.get("timeshift_cam_imu") or 0.0)

    frame_indices = np.array(sorted(frame_stamp_by_index))
    frame_stamps_ns = np.array([int(round(frame_stamp_by_index[index] * 1e9))
                                for index in frame_indices], dtype=np.int64)
    order = np.argsort(frame_stamps_ns)
    frame_indices, frame_stamps_ns = frame_indices[order], frame_stamps_ns[order]
    epoch_ns = int(frame_stamps_ns[0])

    timeus, gyro_xyz, accel_xyz = nora_bin.risi_to_imu(cached["risi"])
    imu_ns = nora_bin.timeus_to_companion_ns(timeus, bridge) - int(round(timeshift_s * 1e9))
    samples = np.column_stack([gyro_xyz, accel_xyz])
    imu_order = np.argsort(imu_ns)
    imu_ns, samples = imu_ns[imu_order].astype(np.int64), samples[imu_order]
    keep_unique = np.concatenate(([True], np.diff(imu_ns) > 0))
    imu_ns, samples = imu_ns[keep_unique], samples[keep_unique]
    window = ((imu_ns >= frame_stamps_ns[0] - IMU_MARGIN_NS)
              & (imu_ns <= frame_stamps_ns[-1] + IMU_MARGIN_NS))
    imu_ns, samples = imu_ns[window], samples[window]
    if len(imu_ns) < 100:
        raise SystemExit(f"IMU window empty ({len(imu_ns)} rows) - bridge or BIN mismatch")
    imu_hz = 1e9 / float(np.median(np.diff(imu_ns)))

    keep_frames = (frame_stamps_ns >= imu_ns[0]) & (frame_stamps_ns <= imu_ns[-1])
    frame_indices, frame_stamps_ns = frame_indices[keep_frames], frame_stamps_ns[keep_frames]
    fps = 1e9 / float(np.median(np.diff(frame_stamps_ns)))

    imu_dir = os.path.join(out_dir, "mav0", "imu0")
    os.makedirs(imu_dir, exist_ok=True)
    write_imu_csv(os.path.join(imu_dir, "data.csv"), imu_ns, samples, epoch_ns)
    written = emit_frames(scan_row, frame_stamps_ns, frame_indices, out_dir, epoch_ns)
    with open(os.path.join(out_dir, "cam0_times.txt"), "w") as handle:
        handle.write("\n".join(str(int(stamp) - epoch_ns) for stamp in frame_stamps_ns) + "\n")
    span_s = (frame_stamps_ns[-1] - frame_stamps_ns[0]) / 1e9
    print(f"  frames: {written}/{len(frame_stamps_ns)} over {span_s:.1f}s ({fps:.1f} fps); "
          f"imu: {len(imu_ns)} rows ~{imu_hz:.0f} Hz shift {-timeshift_s * 1e3:+.2f} ms")

    baro_keep = cached["baro"]["inst"] == 0
    baro_ns = nora_bin.timeus_to_companion_ns(cached["baro"]["timeus"][baro_keep], bridge)
    in_span = (baro_ns >= imu_ns[0]) & (baro_ns <= imu_ns[-1])
    write_reference(os.path.join(out_dir, "ref_baro.csv"), "#timestamp [ns],alt_m",
                    baro_ns[in_span], [cached["baro"]["alt"][baro_keep][in_span]], epoch_ns)
    rfnd_keep = cached["rfnd"]["stat"] == 4 if cached["rfnd"]["dist"].size else np.zeros(0, bool)
    if rfnd_keep.any():
        rfnd_ns = nora_bin.timeus_to_companion_ns(cached["rfnd"]["timeus"][rfnd_keep], bridge)
        in_span = (rfnd_ns >= imu_ns[0]) & (rfnd_ns <= imu_ns[-1])
        write_reference(os.path.join(out_dir, "ref_rfnd.csv"), "#timestamp [ns],dist_m",
                        rfnd_ns[in_span], [cached["rfnd"]["dist"][rfnd_keep][in_span]], epoch_ns)
    att = np.load(os.path.join(cache_dir, "hist_att.npz"))
    if att["timeus"].size:
        att_ns = nora_bin.timeus_to_companion_ns(att["timeus"], bridge)
        in_span = (att_ns >= imu_ns[0]) & (att_ns <= imu_ns[-1])
        write_reference(os.path.join(out_dir, "ref_att.csv"), "#timestamp [ns],roll_rad,pitch_rad",
                        att_ns[in_span], [np.radians(att["roll"][in_span]),
                                          np.radians(att["pitch"][in_span])], epoch_ns)
    gps1 = (cached["gps"]["inst"] == 1) & (cached["gps"]["status"] >= 3)
    if gps1.any():
        gps_ns = nora_bin.timeus_to_companion_ns(cached["gps"]["timeus"][gps1], bridge)
        in_span = (gps_ns >= imu_ns[0]) & (gps_ns <= imu_ns[-1])
        write_reference(os.path.join(out_dir, "ref_gps1.csv"),
                        "#timestamp [ns],lat_deg,lon_deg,alt_m",
                        gps_ns[in_span], [cached["gps"]["lat"][gps1][in_span],
                                          cached["gps"]["lng"][gps1][in_span],
                                          cached["gps"]["alt"][gps1][in_span]], epoch_ns)

    import cv2
    sample = None
    for probe_index in (len(frame_stamps_ns) // 2, 0, len(frame_stamps_ns) - 1):
        sample_name = f"{int(frame_stamps_ns[probe_index]) - epoch_ns}.png"
        sample = cv2.imread(os.path.join(out_dir, "mav0", "cam0", "data", sample_name),
                            cv2.IMREAD_GRAYSCALE)
        if sample is not None:
            break
    if sample is None:
        raise SystemExit("no emitted frame readable for geometry probe")
    config_path, out_width, out_height = write_config(
        flight_key, calibration, hydra_camera, fps, imu_hz,
        stored_size=(sample.shape[1], sample.shape[0]))
    meta = dict(flight_key=flight_key, session=session_path, storage=storage,
                bin=scan_row["bin"], epoch_ns=epoch_ns,
                bridge=dict(source=bridge_source, slope=bridge.slope,
                            k_seconds=bridge.k_seconds, median_abs_residual_ms=residual_ms),
                timeshift_cam_imu_s=timeshift_s, n_frames=int(len(frame_stamps_ns)),
                n_imu=int(len(imu_ns)), imu_hz_median=imu_hz, fps_median=fps,
                image_size=[out_width, out_height], config=config_path,
                clock="companion Unix ns REBASED by epoch_ns")
    with open(os.path.join(out_dir, "dataset_meta.json"), "w") as handle:
        json.dump(meta, handle, indent=2)
    print(f"  -> {out_dir}  config={config_path}")


def binsel_rows(binsel_csv):
    """Adapt bin_session_selection/final_selected_sessions.csv rows (pre-matched
    session<->BIN correspondence, gps0-based selection) to convert_flight() inputs."""
    adapted = []
    with open(binsel_csv, newline="") as handle:
        for row in csv.DictReader(handle):
            if row.get("final_selected", "True") != "True":
                continue
            image_source = row["image_source"]
            if "::" in image_source:
                session_path, storage_tag = image_source.split("::")[0], "session_zip"
            else:
                session_path, storage_tag = os.path.dirname(image_source), "session_dir"
            date_drone = re.search(r"/flights/([^/]+)/([^/]+)/", row["bin_path"])
            adapted.append(dict(
                flight_key=row["image_session_dir"], session_path=session_path,
                image_source=storage_tag, bin=row["bin_path"],
                date=date_drone.group(1) if date_drone else "?",
                drone=date_drone.group(2) if date_drone else "?"))
    return adapted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flight", help="substring of flight_key to convert")
    parser.add_argument("--all-runnable", action="store_true")
    parser.add_argument("--binsel-csv", help="final_selected_sessions.csv to convert from")
    parser.add_argument("--skip-existing", action="store_true",
                        help="skip flights already present in datasets-nora")
    args = parser.parse_args()
    if args.binsel_csv:
        rows = binsel_rows(args.binsel_csv)
    else:
        with open(SCAN_CSV, newline="") as handle:
            rows = [row for row in csv.DictReader(handle) if row["verdict"] == "RUNNABLE"]
    if args.flight:
        rows = [row for row in rows if args.flight in row["flight_key"]]
    if args.skip_existing:
        rows = [row for row in rows
                if not os.path.isfile(os.path.join(DATASETS_ROOT, row["flight_key"],
                                                   "dataset_meta.json"))]
    if not rows:
        raise SystemExit("no matching flights to convert")
    if not (args.all_runnable or args.flight or args.binsel_csv):
        raise SystemExit("pass --flight <key>, --all-runnable or --binsel-csv")
    failures = []
    for row in rows:
        try:
            convert_flight(row)
        except (SystemExit, Exception) as error:  # noqa: BLE001 - batch must survive one flight
            import traceback
            if not isinstance(error, SystemExit):
                traceback.print_exc()
            print(f"  FAILED {row['flight_key']}: {error}")
            failures.append((row["flight_key"], str(error)))
    if failures:
        print(f"\n{len(failures)} failures:")
        for flight_key, reason in failures:
            print(f"  {flight_key}: {reason}")


if __name__ == "__main__":
    main()
