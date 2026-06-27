import cv2
import numpy as np
import cv2.aruco as aruco
import argparse
import sys

from use_calibration import load_calibration, create_undistort_maps, undistort_frame


MARKER_CENTERS_CM = {
    0: (0.0,   0.0),
    1: (23.0, 15.0),
    2: (25,  -9),
    3: (47.5,  -0.5),
}

MARKER_SIZE_CM = 10.0

TABLE_MARKER_IDS = [0, 1, 2, 3]


def detect_aruco(frame, aruco_dict, aruco_params):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    return corners, ids


def pair_corners_to_world(pixel_corners, world_center_cm, marker_size_cm,
                          x_axis_pixels, y_axis_pixels):
    # Tricky part: ArUco returns the 4 corners in the marker's own printed order, which
    # changes when the marker is rotated on the table. To match each pixel corner to the
    # correct world corner, we look at which side of the marker center it falls on along
    # the table's +X and +Y directions, and pick the world corner in that same quadrant.
    half = marker_size_cm / 2.0
    cx, cy = world_center_cm

    world_options = {
        ('+', '+'): (cx + half, cy + half),
        ('+', '-'): (cx + half, cy - half),
        ('-', '+'): (cx - half, cy + half),
        ('-', '-'): (cx - half, cy - half),
    }

    marker_center_px = pixel_corners.mean(axis=0)

    result = []
    used_keys = set()
    for px_corner in pixel_corners:
        v = px_corner - marker_center_px

        proj_x = np.dot(v, x_axis_pixels)
        proj_y = np.dot(v, y_axis_pixels)

        sx = '+' if proj_x >= 0 else '-'
        sy = '+' if proj_y >= 0 else '-'

        key = (sx, sy)
        if key in used_keys:
            best_key = None
            best_score = -1e18
            for cand_key, _ in world_options.items():
                if cand_key in used_keys:
                    continue
                csx, csy = cand_key
                score = (proj_x if csx == '+' else -proj_x) + (proj_y if csy == '+' else -proj_y)
                if score > best_score:
                    best_score = score
                    best_key = cand_key
            key = best_key

        used_keys.add(key)
        result.append(world_options[key])

    return result


def main():
    parser = argparse.ArgumentParser(description="Homography calibration for Camera 1")
    parser.add_argument('--camera-index', type=int, default=0,
                        help='Camera index for Camera 1 (overhead)')
    parser.add_argument('--calibration-file', type=str, default='calibration_cam1.npz',
                        help='Lens calibration file')
    parser.add_argument('--output-file', type=str, default='homography_cam1.npy',
                        help='Where to save the homography matrix')
    args = parser.parse_args()

    print(f"\n[1/6] Loading lens calibration from {args.calibration_file}...")
    cam_matrix, dist_coeffs, image_size = load_calibration(args.calibration_file)
    map1, map2, _ = create_undistort_maps(cam_matrix, dist_coeffs, image_size, crop=True)

    print(f"\n[2/6] Opening camera index {args.camera_index}...")
    cap = cv2.VideoCapture(args.camera_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    if not cap.isOpened():
        print(f"ERROR: Could not open camera index {args.camera_index}")
        sys.exit(1)

    for _ in range(10):
        cap.read()

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    print(f"Place markers {TABLE_MARKER_IDS} flat on the table.")
    print("Press SPACE when all 4 are clearly detected. Q to quit.")

    window_name = "Cam1 Homography - SPACE=capture  Q=quit"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 540)

    captured_frame = None
    while True:
        ret, raw = cap.read()
        if not ret:
            continue
        frame = undistort_frame(raw, map1, map2)
        corners, ids = detect_aruco(frame, aruco_dict, aruco_params)

        preview = frame.copy()
        if ids is not None:
            aruco.drawDetectedMarkers(preview, corners, ids)
            seen = sorted(set(ids.flatten().tolist()))
            missing = [m for m in TABLE_MARKER_IDS if m not in seen]
            if not missing:
                text = f"All 4 markers detected: {seen}  -  press SPACE"
                color = (0, 255, 0)
            else:
                text = f"Detected: {seen}  Missing: {missing}"
                color = (0, 165, 255)
        else:
            text = "No markers detected"
            color = (0, 0, 255)
        cv2.putText(preview, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        cv2.imshow(window_name, preview)
        key = cv2.waitKey(1) & 0xFF
        if key == 32:
            captured_frame = frame.copy()
            break
        elif key in (ord('q'), ord('Q')):
            cap.release()
            cv2.destroyAllWindows()
            print("Quit by user.")
            sys.exit(0)

    cap.release()
    cv2.destroyAllWindows()

    print(f"\n[3/6] Detecting markers on captured frame...")
    corners, ids = detect_aruco(captured_frame, aruco_dict, aruco_params)
    if ids is None:
        print("ERROR: No markers detected.")
        sys.exit(1)

    found = ids.flatten().tolist()
    missing = [m for m in TABLE_MARKER_IDS if m not in found]
    if missing:
        print(f"ERROR: Missing required markers: {missing}")
        sys.exit(1)

    pixel_corners_by_id = {}
    for mid in TABLE_MARKER_IDS:
        idx = found.index(mid)
        pixel_corners_by_id[mid] = corners[idx][0]
    print(f"  All required markers detected: {TABLE_MARKER_IDS}")

    print(f"\n[4/6] Computing pixel-space axis directions...")
    center_0_px = pixel_corners_by_id[0].mean(axis=0)
    center_3_px = pixel_corners_by_id[3].mean(axis=0)
    center_1_px = pixel_corners_by_id[1].mean(axis=0)

    x_axis_px = center_3_px - center_0_px
    x_axis_px = x_axis_px / np.linalg.norm(x_axis_px)

    y_axis_px = np.array([-x_axis_px[1], x_axis_px[0]])

    v_01 = center_1_px - center_0_px
    if np.dot(v_01, y_axis_px) < 0:
        y_axis_px = -y_axis_px

    print(f"  +X (in pixels): {x_axis_px}")
    print(f"  +Y (in pixels): {y_axis_px}")

    print(f"\n[5/6] Building 16 pixel-to-cm correspondences...")
    pixel_points = []
    world_points = []
    for mid in TABLE_MARKER_IDS:
        px_corners = pixel_corners_by_id[mid]
        world_corners = pair_corners_to_world(
            px_corners,
            MARKER_CENTERS_CM[mid],
            MARKER_SIZE_CM,
            x_axis_px,
            y_axis_px,
        )
        for i in range(4):
            pixel_points.append(px_corners[i])
            world_points.append(world_corners[i])

    pixel_points = np.array(pixel_points, dtype=np.float32)
    world_points = np.array(world_points, dtype=np.float32)

    print(f"\n[6/6] Computing homography...")
    H, mask = cv2.findHomography(pixel_points, world_points, cv2.RANSAC, 1.0)
    if H is None:
        print("ERROR: cv2.findHomography failed.")
        sys.exit(1)

    inliers = int(mask.sum())
    print(f"  RANSAC inliers: {inliers}/{len(mask)}")

    reprojected = cv2.perspectiveTransform(
        pixel_points.reshape(-1, 1, 2), H
    ).reshape(-1, 2)
    errors_cm = np.linalg.norm(reprojected - world_points, axis=1)
    mean_err = float(np.mean(errors_cm))
    max_err = float(np.max(errors_cm))
    print(f"\n  Validation against ruler-measured ground truth:")
    print(f"    Mean reprojection error: {mean_err:.3f} cm")
    print(f"    Max  reprojection error: {max_err:.3f} cm")

    print(f"\n  Per-marker mean error:")
    for i, mid in enumerate(TABLE_MARKER_IDS):
        marker_errors = errors_cm[i*4:(i+1)*4]
        print(f"    ID {mid}: mean={marker_errors.mean():.3f} cm, max={marker_errors.max():.3f} cm")

    if mean_err < 0.3:
        verdict = "EXCELLENT"
    elif mean_err < 0.7:
        verdict = "GOOD"
    elif mean_err < 1.5:
        verdict = "ACCEPTABLE"
    else:
        verdict = "POOR - check ruler measurements"
    print(f"\n  Verdict: {verdict}")

    np.save(args.output_file, H)
    cv2.imwrite("homography_cam1_reference.jpg", captured_frame)
    print(f"\n  Saved homography to:        {args.output_file}")
    print(f"  Saved reference image to:   homography_cam1_reference.jpg")
    print("\nHomography matrix H (pixels -> cm):")
    print(H)
    print("\nDone!")


if __name__ == '__main__':
    main()
