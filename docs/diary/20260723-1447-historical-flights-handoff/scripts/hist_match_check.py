#!/usr/bin/env python3
"""Verify the user's 300x225 front-end requirements on a converted historical dataset:

  (a) >= --min-features ORB detections per frame (extractor mirror: OpenCV ORB with the
      config's nFeatures / minThFAST; the authoritative count is ORB-SLAM3's own
      frame_stats.csv `detections` column - compare both),
  (b) >= --min-matches RANSAC-verified matches between each frame and AT LEAST ONE of its
      4 neighbours (offsets 1..4, both directions counted for the frame-level verdict).

Matches: BF-Hamming knn + Lowe 0.75, points undistorted with the dataset's KB4 fisheye
calibration (from the generated config), RANSAC homography inliers = correct matches
(quasi-planar ground from altitude).

Writes <out>/pair_matches.csv, <out>/frame_verdicts.csv and prints a summary.
"""
import argparse
import csv
import os
import re

import cv2
import numpy as np


def read_orb_config(config_path):
    values = {}
    with open(config_path) as handle:
        for line in handle:
            match = re.match(r"([A-Za-z0-9._]+):\s*([-0-9.e+]+)\s*$", line.strip())
            if match:
                values[match.group(1)] = float(match.group(2))
    intrinsic_matrix = np.array([[values["Camera1.fx"], 0.0, values["Camera1.cx"]],
                                 [0.0, values["Camera1.fy"], values["Camera1.cy"]],
                                 [0.0, 0.0, 1.0]])
    distortion = np.array([values["Camera1.k1"], values["Camera1.k2"],
                           values["Camera1.k3"], values["Camera1.k4"]])
    # Mirror ORB-SLAM3's config-side upscale: images resized to newWidth/newHeight and
    # K scaled accordingly (KB distortion acts on the incidence angle - unchanged).
    new_size = None
    if "Camera.newWidth" in values:
        new_size = (int(values["Camera.newWidth"]), int(values["Camera.newHeight"]))
        upscale = values["Camera.newWidth"] / values["Camera.width"]
        intrinsic_matrix[:2, :] *= upscale
    return (intrinsic_matrix, distortion, int(values["ORBextractor.nFeatures"]),
            int(values["ORBextractor.minThFAST"]),
            float(values.get("ORBextractor.scaleFactor", 1.2)),
            int(values.get("ORBextractor.nLevels", 8)), new_size)


def load_frames(dataset_dir):
    with open(os.path.join(dataset_dir, "cam0_times.txt")) as handle:
        stamps = [int(line.strip()) for line in handle if line.strip()]
    image_dir = os.path.join(dataset_dir, "mav0", "cam0", "data")
    return [(stamp, os.path.join(image_dir, f"{stamp}.png")) for stamp in stamps]


def ransac_inliers(kp_a, kp_b, matches, intrinsic_matrix, distortion):
    if len(matches) < 8:
        return len(matches)
    points_a = np.float32([kp_a[match.queryIdx].pt for match in matches]).reshape(-1, 1, 2)
    points_b = np.float32([kp_b[match.trainIdx].pt for match in matches]).reshape(-1, 1, 2)
    undist_a = cv2.fisheye.undistortPoints(points_a, intrinsic_matrix, distortion,
                                           P=intrinsic_matrix)
    undist_b = cv2.fisheye.undistortPoints(points_b, intrinsic_matrix, distortion,
                                           P=intrinsic_matrix)
    _homography, inlier_mask = cv2.findHomography(undist_a, undist_b, cv2.RANSAC, 3.0)
    if inlier_mask is None:
        return 0
    return int(inlier_mask.sum())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--min-features", type=int, default=10000)
    parser.add_argument("--min-matches", type=int, default=500)
    parser.add_argument("--max-offset", type=int, default=4)
    parser.add_argument("--clahe", type=float, default=3.0)
    parser.add_argument("--stride", type=int, default=1,
                        help="evaluate every Nth frame as pair start (neighbours always dense)")
    parser.add_argument("--limit", type=int, default=0, help="only first N frames (0=all)")
    args = parser.parse_args()

    (intrinsic_matrix, distortion, n_features, min_th_fast,
     scale_factor, n_levels, new_size) = read_orb_config(args.config)
    frames = load_frames(args.dataset)
    if args.limit:
        frames = frames[:args.limit]
    os.makedirs(args.out, exist_ok=True)

    orb = cv2.ORB_create(nfeatures=n_features, scaleFactor=scale_factor, nlevels=n_levels,
                         fastThreshold=min_th_fast)
    clahe = cv2.createCLAHE(clipLimit=args.clahe, tileGridSize=(8, 8)) if args.clahe > 0 else None
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    window = {}  # frame index -> (keypoints, descriptors)
    detection_counts = []
    pair_rows = []
    best_by_frame = {}

    def detect(frame_index):
        if frame_index in window:
            return window[frame_index]
        image = cv2.imread(frames[frame_index][1], cv2.IMREAD_GRAYSCALE)
        if image is None:
            window[frame_index] = ([], None)
            return window[frame_index]
        if new_size is not None:
            image = cv2.resize(image, new_size, interpolation=cv2.INTER_LINEAR)
        if clahe is not None:
            image = clahe.apply(image)
        keypoints, descriptors = orb.detectAndCompute(image, None)
        window[frame_index] = (keypoints, descriptors)
        detection_counts.append((frame_index, len(keypoints)))
        return window[frame_index]

    total = len(frames)
    for start in range(0, total, args.stride):
        kp_a, desc_a = detect(start)
        for offset in range(1, args.max_offset + 1):
            neighbour = start + offset
            if neighbour >= total:
                continue
            kp_b, desc_b = detect(neighbour)
            correct = 0
            if desc_a is not None and desc_b is not None:
                knn = matcher.knnMatch(desc_a, desc_b, k=2)
                good = [pair[0] for pair in knn
                        if len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance]
                correct = ransac_inliers(kp_a, kp_b, good, intrinsic_matrix, distortion)
            pair_rows.append((start, neighbour, offset, correct))
            best_by_frame[start] = max(best_by_frame.get(start, 0), correct)
            best_by_frame[neighbour] = max(best_by_frame.get(neighbour, 0), correct)
        # slide the window: drop everything older than max_offset behind
        for stale in [key for key in window if key < start - args.max_offset]:
            del window[stale]
        if start % 200 == 0 and start:
            done_counts = [count for _idx, count in detection_counts]
            print(f"  {start}/{total} frames; det median {int(np.median(done_counts))}; "
                  f"pairs {len(pair_rows)}", flush=True)

    with open(os.path.join(args.out, "pair_matches.csv"), "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame_a", "frame_b", "offset", "correct_matches"])
        writer.writerows(pair_rows)
    with open(os.path.join(args.out, "frame_verdicts.csv"), "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "best_neighbour_matches", "pass"])
        for frame_index in sorted(best_by_frame):
            best = best_by_frame[frame_index]
            writer.writerow([frame_index, best, int(best >= args.min_matches)])

    counts = np.array([count for _idx, count in detection_counts])
    frames_pass_features = int((counts >= args.min_features).sum())
    best_values = np.array([best_by_frame[key] for key in sorted(best_by_frame)])
    frames_pass_matches = int((best_values >= args.min_matches).sum())
    per_offset = {offset: np.array([row[3] for row in pair_rows if row[2] == offset])
                  for offset in range(1, args.max_offset + 1)}
    print(f"\nDATASET {args.dataset}")
    print(f"features : n={len(counts)} median={int(np.median(counts))} "
          f"min={int(counts.min())} p5={int(np.percentile(counts, 5))} "
          f">= {args.min_features}: {frames_pass_features}/{len(counts)} "
          f"({100.0 * frames_pass_features / len(counts):.1f}%)")
    for offset, values in per_offset.items():
        if len(values):
            print(f"offset {offset}  : median={int(np.median(values))} "
                  f"p5={int(np.percentile(values, 5))} min={int(values.min())}")
    print(f"matches  : frames with >= {args.min_matches} to >=1 of {args.max_offset} "
          f"neighbours: {frames_pass_matches}/{len(best_values)} "
          f"({100.0 * frames_pass_matches / len(best_values):.1f}%)")


if __name__ == "__main__":
    main()
