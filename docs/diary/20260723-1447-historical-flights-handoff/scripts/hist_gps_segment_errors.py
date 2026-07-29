#!/usr/bin/env python3
"""Per-1-minute-segment trajectory errors vs GPS[0] for the hist_baro2 flights.

For every flight with >80% coverage whose GPS[0] is unspoofed for >=60 s after takeoff:
  * takeoff = the session's LOG_DRONE.txt "Flight is started" stamp (local wall clock;
    the local->UTC offset is recovered by rounding (naive_log_time - dataset epoch) to
    the nearest 30 min - the message coincides with the first recorded frame to ~1 s);
  * spoof mask: GPS[0] fix is UNSPOOFED where it agrees horizontally (<30 m) with an
    interpolated GPS[1] fix; without GPS[1] fixes, a plausibility fallback
    (3D fix, >=6 sats, inter-fix speed < 40 m/s);
  * global alignment: 3D Umeyama (with scale) on the first 30 s after takeoff between
    the SLAM trajectory (time-interpolated GPS[0] ENU correspondence) and GPS[0];
  * per 60-s segment: a fresh Umeyama on the globally-aligned SLAM segment reports the
    segment's residual errors - horizontal translation (xy centroid offset, m),
    rotation (angle of the segment rotation, deg), scale (|s-1| in %);
  * output: per-flight percentiles (p10/p25/p50/p75/p90) over its segments.
"""
import csv
import glob
import json
import os
import re
from datetime import datetime, timezone

import numpy as np

ROOT = "/home/kmro/praca/dev/orbslam3-eval"
BINCACHE_ROOT = os.path.join(ROOT, "eval_out", "bincache")
SUMMARY = os.path.join(ROOT, "runs", "det_campaign_summary.csv")
MIN_COVERAGE_PCT = 80.0
SPOOF_AGREE_M = 30.0
ALIGN_WINDOW_S = 30.0
SEGMENT_S = 60.0
EARTH_RADIUS_M = 6371000.0
# WMM magnetic declination at the test field (50.41N 29.94E), 2025/2026: ~ +9.3 deg East.
DECLINATION_DEG = 9.3


def load_flight_rows(batch="hist_baro2"):
    with open(SUMMARY, newline="") as handle:
        rows = [row for row in csv.DictReader(handle) if row["batch"] == batch]
    return [row for row in rows if float(row["cov_pct"]) > MIN_COVERAGE_PCT]


def flight_key_from_tag(tag):
    stem = tag.split("_", 1)[1]  # strip the batch prefix (hb2_/m05_/...)
    matches = glob.glob(os.path.join(ROOT, "datasets-nora", stem + "_*"))
    if len(matches) != 1:
        raise SystemExit(f"{tag}: ambiguous dataset dirs {matches}")
    return os.path.basename(matches[0])


def takeoff_rebased_s(meta):
    """'Flight is started' stamp from LOG_DRONE.txt, converted to rebased seconds."""
    log_path = os.path.join(meta["session"], "logs", "LOG_DRONE.txt")
    if not os.path.isfile(log_path):
        return None, "no LOG_DRONE.txt"
    started = None
    with open(log_path, errors="replace") as handle:
        for line in handle:
            if "Flight is started" in line:
                match = re.match(r"\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}),(\d{3})\]", line)
                if match:
                    naive = datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S")
                    started = (naive.replace(tzinfo=timezone.utc).timestamp()
                               + int(match.group(2)) / 1e3)
                break
    if started is None:
        return None, "no 'Flight is started' message"
    epoch_unix = meta["epoch_ns"] / 1e9
    tz_offset = round((started - epoch_unix) / 1800.0) * 1800.0
    return started - tz_offset - epoch_unix, f"tz_offset={tz_offset / 3600:+.1f}h"


def gps_enu(meta, flight_key):
    """GPS[0]/GPS[1] fixes as (rebased_s, east_m, north_m, up_m) + spoof-check inputs."""
    cache_key = re.sub(r"[^A-Za-z0-9]", "_", meta["bin"]) + f"_{os.path.getsize(meta['bin'])}"
    gps = dict(np.load(os.path.join(BINCACHE_ROOT, cache_key, "hist_gps.npz")))
    bridge = meta["bridge"]
    stamps_rebased = ((np.round(bridge["slope"] * 1000.0 * gps["timeus"].astype(np.float64))
                       + round(bridge["k_seconds"] * 1e9)) - meta["epoch_ns"]) / 1e9
    lat_ref = np.median(gps["lat"][gps["status"] >= 3])
    lon_ref = np.median(gps["lng"][gps["status"] >= 3])
    east = np.radians(gps["lng"] - lon_ref) * EARTH_RADIUS_M * np.cos(np.radians(lat_ref))
    north = np.radians(gps["lat"] - lat_ref) * EARTH_RADIUS_M
    result = {}
    for instance in (0, 1):
        keep = (gps["inst"] == instance) & (gps["status"] >= 3)
        order = np.argsort(stamps_rebased[keep])
        result[instance] = dict(
            t=stamps_rebased[keep][order], east=east[keep][order], north=north[keep][order],
            up=gps["alt"][keep][order], nsats=gps["nsats"][keep][order])
    return result


def mag_heading_series(meta):
    """True-north compass heading of the drone from BIN MAG (instance 0), tilt-compensated
    with ATT roll/pitch, declination-corrected. Returns (rebased_s, heading_rad)."""
    cache_key = re.sub(r"[^A-Za-z0-9]", "_", meta["bin"]) + f"_{os.path.getsize(meta['bin'])}"
    cache_dir = os.path.join(BINCACHE_ROOT, cache_key)
    mag = dict(np.load(os.path.join(cache_dir, "hist_mag.npz")))
    att = dict(np.load(os.path.join(cache_dir, "hist_att.npz")))
    keep = mag["inst"] == 0
    if keep.sum() < 8 or att["timeus"].size < 8:
        return np.zeros(0), np.zeros(0)
    bridge = meta["bridge"]

    def rebase(timeus):
        return ((np.round(bridge["slope"] * 1000.0 * timeus.astype(np.float64))
                 + round(bridge["k_seconds"] * 1e9)) - meta["epoch_ns"]) / 1e9

    mag_t = rebase(mag["timeus"][keep])
    att_t = rebase(att["timeus"])
    roll = np.interp(mag_t, att_t, np.radians(att["roll"]))
    pitch = np.interp(mag_t, att_t, np.radians(att["pitch"]))
    mag_x, mag_y, mag_z = (mag[axis][keep].astype(np.float64)
                           for axis in ("magx", "magy", "magz"))
    # Body (x fwd, y right, z down) -> level frame: m_level = Ry(pitch) @ Rx(roll) @ m.
    level_x = (np.cos(pitch) * mag_x + np.sin(pitch) * np.sin(roll) * mag_y
               + np.sin(pitch) * np.cos(roll) * mag_z)
    level_y = np.cos(roll) * mag_y - np.sin(roll) * mag_z
    heading = np.arctan2(-level_y, level_x) + np.radians(DECLINATION_DEG)
    return mag_t, heading


def circular_median(angles_rad):
    return float(np.arctan2(np.median(np.sin(angles_rad)), np.median(np.cos(angles_rad))))


def unspoofed_mask(gps):
    """Boolean mask over GPS[0] fixes; returns (mask, method)."""
    gps0, gps1 = gps[0], gps[1]
    if len(gps1["t"]) >= 8:
        east1 = np.interp(gps0["t"], gps1["t"], gps1["east"])
        north1 = np.interp(gps0["t"], gps1["t"], gps1["north"])
        inside = (gps0["t"] >= gps1["t"][0]) & (gps0["t"] <= gps1["t"][-1])
        distance = np.hypot(gps0["east"] - east1, gps0["north"] - north1)
        return inside & (distance < SPOOF_AGREE_M), "gps1-agreement"
    if len(gps0["t"]) < 8:
        return np.zeros(0, dtype=bool), "no-gps0"
    speed = np.zeros(len(gps0["t"]))
    dt = np.diff(gps0["t"])
    step = np.hypot(np.diff(gps0["east"]), np.diff(gps0["north"]))
    speed[1:] = np.where(dt > 0, step / np.maximum(dt, 1e-3), np.inf)
    return (gps0["nsats"] >= 6) & (speed < 40.0), "plausibility"


def umeyama(source_points, target_points):
    """Similarity target = s*R*source + t. Returns (scale, rotation, translation)."""
    source_mean = source_points.mean(axis=0)
    target_mean = target_points.mean(axis=0)
    source_centered = source_points - source_mean
    target_centered = target_points - target_mean
    covariance = target_centered.T @ source_centered / len(source_points)
    u_matrix, singular, vt_matrix = np.linalg.svd(covariance)
    sign_fix = np.eye(3)
    if np.linalg.det(u_matrix) * np.linalg.det(vt_matrix) < 0:
        sign_fix[2, 2] = -1.0
    rotation = u_matrix @ sign_fix @ vt_matrix
    source_variance = (source_centered ** 2).sum() / len(source_points)
    scale = float(np.trace(np.diag(singular) @ sign_fix) / source_variance)
    translation = target_mean - scale * rotation @ source_mean
    return scale, rotation, translation


def rotation_angle_deg(rotation):
    return float(np.degrees(np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))))


def analyse_flight(row):
    tag = row["tag"]
    flight_key = flight_key_from_tag(tag)
    meta = json.load(open(os.path.join(ROOT, "datasets-nora", flight_key, "dataset_meta.json")))
    takeoff, takeoff_note = takeoff_rebased_s(meta)
    if takeoff is None:
        return dict(flight=flight_key, skip=takeoff_note)
    gps = gps_enu(meta, flight_key)
    mask, spoof_method = unspoofed_mask(gps)
    gps0 = gps[0]

    # Flight length: duration = takeoff -> last recorded frame; horizontal distance from
    # GPS[1] (authentic instance) when present, else the unspoofed GPS[0] fixes.
    times_path = os.path.join(ROOT, "datasets-nora", flight_key, "cam0_times.txt")
    with open(times_path) as handle:
        frame_stamps = [int(line) for line in handle if line.strip()]
    duration_s = frame_stamps[-1] / 1e9 - takeoff
    if duration_s <= 0:
        duration_s = float("nan")
    length_source, length_track = "gps1", gps[1]
    if len(length_track["t"]) < 8:
        keep0 = mask if len(mask) else np.zeros(len(gps0["t"]), dtype=bool)
        length_track = dict(t=gps0["t"][keep0], east=gps0["east"][keep0],
                            north=gps0["north"][keep0])
        length_source = "gps0-unspoofed"
    in_flight = length_track["t"] >= takeoff
    if in_flight.sum() >= 2:
        path_m = float(np.linalg.norm(
            np.column_stack([np.diff(length_track["east"][in_flight]),
                             np.diff(length_track["north"][in_flight])]), axis=1).sum())
    else:
        path_m, length_source = float("nan"), "none"
    flight_note = f"flight {duration_s:.0f}s / {path_m:.0f}m [{length_source}]"
    good_t = gps0["t"][mask]
    after_takeoff = good_t[good_t >= takeoff]
    if len(after_takeoff) < 8 or after_takeoff.max() - takeoff < 60.0:
        gps1 = gps[1]
        offset_note = ""
        if len(gps1["t"]) >= 8:
            window = (gps0["t"] >= takeoff) & (gps0["t"] <= takeoff + 180.0)
            if window.sum():
                east1 = np.interp(gps0["t"][window], gps1["t"], gps1["east"])
                north1 = np.interp(gps0["t"][window], gps1["t"], gps1["north"])
                offset = np.median(np.hypot(gps0["east"][window] - east1,
                                            gps0["north"][window] - north1))
                offset_note = f", median |gps0-gps1| first 3min = {offset:.0f} m"
        return dict(flight=flight_key, skip=f"<60s unspoofed GPS[0] after takeoff "
                                            f"({spoof_method}, "
                                            f"{0 if len(after_takeoff) == 0 else after_takeoff.max() - takeoff:.0f}s"
                                            f"{offset_note}); {flight_note}")

    dataset_symlink = glob.glob(os.path.join(ROOT, "runs", "*", "mono_inertial", tag,
                                             f"f_{tag}.txt"))
    trajectory = np.loadtxt(dataset_symlink[0])
    traj_t, traj_xyz = trajectory[:, 0] / 1e9, trajectory[:, 1:4]  # stamps are rebased ns
    # SLAM body yaw in the (gravity-aligned) SLAM world: heading of the body x-axis.
    quat_x, quat_y, quat_z, quat_w = trajectory[:, 4:8].T
    forward_x = 1.0 - 2.0 * (quat_y * quat_y + quat_z * quat_z)
    forward_y = 2.0 * (quat_x * quat_y + quat_z * quat_w)
    traj_yaw = np.arctan2(forward_y, forward_x)

    good = np.zeros(len(gps0["t"]), dtype=bool)
    good[:len(mask)] = mask
    gps_t = gps0["t"][good]
    gps_xyz = np.column_stack([gps0["east"][good], gps0["north"][good], gps0["up"][good]])

    def correspondences(window_lo, window_hi):
        keep = (traj_t >= window_lo) & (traj_t < window_hi)
        keep &= (traj_t >= gps_t[0]) & (traj_t <= gps_t[-1])
        stamps = traj_t[keep]
        if len(stamps) < 10:
            return None, None
        gps_interp = np.column_stack([np.interp(stamps, gps_t, gps_xyz[:, axis])
                                      for axis in range(3)])
        # No correspondences across spoofed/unspoofed holes: require a real GPS fix
        # within 1.5 s of the trajectory stamp.
        insertion = np.searchsorted(gps_t, stamps)
        left = gps_t[np.clip(insertion - 1, 0, len(gps_t) - 1)]
        right = gps_t[np.clip(insertion, 0, len(gps_t) - 1)]
        ok = np.minimum(np.abs(stamps - left), np.abs(right - stamps)) < 1.5
        if ok.sum() < 10:
            return None, None
        return traj_xyz[keep][ok], gps_interp[ok]

    # The strict [takeoff, takeoff+30] window frequently has NO usable data (SLAM init
    # completes tens of seconds after takeoff; GPS[0] often has early gaps): slide the
    # 30-s alignment window forward to the first placement with enough correspondences.
    # A takeoff-hover window has near-zero baseline and makes the Umeyama scale
    # degenerate (observed 196x): additionally require >=30 m of GPS path in the window.
    align_start, slam_align, gps_align = None, None, None
    slide = takeoff
    while slide + ALIGN_WINDOW_S <= min(traj_t.max(), gps_t.max()):
        candidate_slam, candidate_gps = correspondences(slide, slide + ALIGN_WINDOW_S)
        if candidate_slam is not None and len(candidate_slam) >= 10:
            gps_path_m = float(np.linalg.norm(np.diff(candidate_gps[:, :2], axis=0),
                                              axis=1).sum())
            if gps_path_m >= 30.0:
                align_start, slam_align, gps_align = slide, candidate_slam, candidate_gps
                break
        slide += 10.0
    if slam_align is None:
        return dict(flight=flight_key,
                    skip="no 30s window with traj+unspoofed-GPS overlap and >=30m motion")

    # Rotation from the MAGNETOMETER instead of Umeyama: per-sample difference between
    # the tilt-compensated true-north compass heading (ENU math angle pi/2 - psi) and
    # the SLAM body yaw, circular-median over the align window; yaw-only rotation
    # (both frames are gravity-aligned). Scale/translation: least squares with R fixed.
    mag_t, mag_heading = mag_heading_series(meta)
    if len(mag_t) < 8:
        return dict(flight=flight_key, skip="no usable MAG/ATT data for alignment")
    window_keep = (traj_t >= align_start) & (traj_t < align_start + ALIGN_WINDOW_S)
    stamps = traj_t[window_keep]
    heading_interp = np.arctan2(np.interp(stamps, mag_t, np.sin(mag_heading)),
                                np.interp(stamps, mag_t, np.cos(mag_heading)))
    enu_math_angle = np.pi / 2.0 - heading_interp
    # The SLAM world is gravity-aligned but z-DOWN (climb = negative z; verified: the
    # per-sample mag+slam_yaw sum is constant to ~4 deg while mag-slam wanders 37 deg),
    # so ENU = Rz(yaw_delta) @ diag(1,-1,-1) @ SLAM and the body heading enters negated.
    delta_samples = enu_math_angle + traj_yaw[window_keep]
    yaw_delta = circular_median(delta_samples)
    delta_dev_deg = np.degrees(np.abs(((delta_samples - yaw_delta + np.pi)
                                       % (2.0 * np.pi)) - np.pi))
    if np.median(delta_dev_deg) > 25.0:
        return dict(flight=flight_key,
                    skip=f"mag heading unstable vs SLAM yaw in align window "
                         f"(median dev {np.median(delta_dev_deg):.0f} deg) - "
                         f"compass unreliable on this airframe")
    rotation_g = np.array([[np.cos(yaw_delta), -np.sin(yaw_delta), 0.0],
                           [np.sin(yaw_delta), np.cos(yaw_delta), 0.0],
                           [0.0, 0.0, 1.0]]) @ np.diag([1.0, -1.0, -1.0])
    # The map's gravity is typically tilted a few degrees from true vertical (IMU-init
    # Rwg error; the relative-datum baro edges are tilt-blind and mag constrains only
    # yaw). A yaw-only alignment then leaks horizontal motion into z and biases the
    # least-squares scale low by 10-30%. Estimate the constant tilt from a full-flight
    # free Umeyama and keep ONLY its roll/pitch, overriding its yaw with the mag yaw.
    tilt_deg = 0.0
    slam_all, gps_all = correspondences(takeoff, min(traj_t.max(), gps_t.max()))
    if slam_all is not None and len(slam_all) >= 100:
        gps_path_all = float(np.linalg.norm(np.diff(gps_all[:, :2], axis=0), axis=1).sum())
        if gps_path_all >= 300.0:
            _scale_f, rotation_full, _translation_f = umeyama(slam_all, gps_all)
            up_mapped = rotation_full @ np.array([0.0, 0.0, -1.0])
            tilt_deg = float(np.degrees(np.arccos(np.clip(up_mapped[2], -1.0, 1.0))))
            yaw_full = np.arctan2(rotation_full[1, 0], rotation_full[0, 0])
            correction = yaw_delta - yaw_full
            rotation_g = np.array([[np.cos(correction), -np.sin(correction), 0.0],
                                   [np.sin(correction), np.cos(correction), 0.0],
                                   [0.0, 0.0, 1.0]]) @ rotation_full
    slam_centered = slam_align - slam_align.mean(axis=0)
    gps_centered = gps_align - gps_align.mean(axis=0)
    rotated = (rotation_g @ slam_centered.T).T
    scale_g = float((rotated * gps_centered).sum() / (slam_centered ** 2).sum())
    if not 0.2 <= scale_g <= 5.0:
        return dict(flight=flight_key,
                    skip=f"fixed-rotation scale fit non-physical ({scale_g:.3f})")
    translation_g = gps_align.mean(axis=0) - scale_g * rotation_g @ slam_align.mean(axis=0)
    takeoff_window = (mag_t >= takeoff) & (mag_t < takeoff + ALIGN_WINDOW_S)
    mag_takeoff_deg = (np.degrees(circular_median(mag_heading[takeoff_window])) % 360.0
                       if takeoff_window.sum() else float("nan"))

    segments = []
    segment_start = takeoff
    horizon = min(traj_t.max(), gps_t.max())
    while segment_start + SEGMENT_S <= horizon + 1.0:
        slam_seg, gps_seg = correspondences(segment_start, segment_start + SEGMENT_S)
        segment_index = int((segment_start - takeoff) // SEGMENT_S)
        segment_start += SEGMENT_S
        if slam_seg is None or len(slam_seg) < 30:
            continue
        aligned = (scale_g * (rotation_g @ slam_seg.T)).T + translation_g
        scale_s, rotation_s, _translation_s = umeyama(aligned, gps_seg)
        horizontal = float(np.linalg.norm(gps_seg.mean(axis=0)[:2] - aligned.mean(axis=0)[:2]))
        segment_path_m = float(np.linalg.norm(np.diff(gps_seg[:, :2], axis=0), axis=1).sum())
        segments.append(dict(index=segment_index, horiz_m=horizontal,
                             rot_deg=rotation_angle_deg(rotation_s),
                             scale_pct=abs(scale_s - 1.0) * 100.0, n=len(slam_seg),
                             scale_abs=scale_s * scale_g,
                             path_m=segment_path_m,
                             scale_valid=segment_path_m >= 200.0))
    if not segments:
        return dict(flight=flight_key, skip="no complete 60s segments with GPS+traj")
    return dict(flight=flight_key, spoof_method=spoof_method, takeoff=takeoff,
                takeoff_note=takeoff_note, align_scale=scale_g, align_start=align_start,
                unspoofed_span_s=float(after_takeoff.max() - takeoff), segments=segments,
                flight_note=flight_note, yaw_delta_deg=np.degrees(yaw_delta),
                mag_takeoff_deg=mag_takeoff_deg, tilt_deg=tilt_deg)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch", default="hist_baro2")
    args = parser.parse_args()
    percentile_marks = (10, 25, 50, 75, 90)
    for row in sorted(load_flight_rows(args.batch), key=lambda item: item["tag"]):
        result = analyse_flight(row)
        if "skip" in result:
            print(f"{result['flight']:<44} SKIP: {result['skip']}")
            continue
        segments = result["segments"]
        print(f"\n[{row['tag']}] {result['flight']}  (cov {row['cov_pct']}%, {result['flight_note']}, "
              f"spoof-check {result['spoof_method']}, takeoff t={result['takeoff']:.1f}s "
              f"{result['takeoff_note']}, unspoofed {result['unspoofed_span_s']:.0f}s, "
              f"align window [{result['align_start']:.0f},"
              f"{result['align_start'] + ALIGN_WINDOW_S:.0f}]s MAG-yaw "
              f"{result['yaw_delta_deg']:+.1f}deg (takeoff hdg {result['mag_takeoff_deg']:.0f}deg, "
              f"decl {DECLINATION_DEG}deg) scale {result['align_scale']:.4f}, "
              f"{len(segments)} segments)")
        print(f"  map tilt vs vertical: {result['tilt_deg']:.1f} deg (compensated)")
        for metric, unit in (("horiz_m", "m"), ("rot_deg", "deg")):
            values = np.array([segment[metric] for segment in segments])
            marks = " ".join(f"p{p}={np.percentile(values, p):.2f}"
                             for p in percentile_marks)
            print(f"  {metric:<10} [{unit:>3}] {marks}")
        scale_ok = [segment for segment in segments if segment["scale_valid"]]
        if scale_ok:
            values = np.array([segment["scale_pct"] for segment in scale_ok])
            marks = " ".join(f"p{p}={np.percentile(values, p):.2f}" for p in percentile_marks)
            absolutes = np.array([segment["scale_abs"] for segment in scale_ok])
            in_band = int(((absolutes > 0.95) & (absolutes < 1.1)).sum())
            print(f"  scale_pct  [  %] {marks}  (over {len(scale_ok)} motion-rich segments; "
                  f"abs scale in (0.95,1.1): {in_band}/{len(scale_ok)})")
        degenerate = len(segments) - len(scale_ok)
        if degenerate:
            print(f"  ({degenerate} low-motion segments <200 m GPS path: scale not evaluable)")
        per_seg = " ".join(
            f"#{s['index']}:{s['horiz_m']:.1f}m/{s['rot_deg']:.1f}d/"
            + (f"s={s['scale_abs']:.3f}" if s["scale_valid"] else "s=deg")
            for s in segments)
        print(f"  segments: {per_seg}")


if __name__ == "__main__":
    main()
