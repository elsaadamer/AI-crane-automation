import cv2
import numpy as np
import glob
import os
import argparse
import json


CHECKERBOARD_COLS = 9
CHECKERBOARD_ROWS = 6

CHARUCO_COLS = 8
CHARUCO_ROWS = 11
CHARUCO_DICT = cv2.aruco.DICT_4X4_50

SQUARE_SIZE_MM = 22.0
MARKER_SIZE_MM = 18.75


def detect_checkerboard(gray_image, cols, rows):
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK
    )

    found, corners = cv2.findChessboardCorners(gray_image, (cols, rows), flags)

    if found:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(
            gray_image, corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=criteria
        )

    return found, corners


def detect_charuco(gray_image, board, charuco_detector):
    charuco_corners, charuco_ids, marker_corners, marker_ids = \
        charuco_detector.detectBoard(gray_image)

    num_markers = len(marker_corners) if marker_corners is not None and len(marker_corners) > 0 else 0
    num_corners = len(charuco_corners) if charuco_corners is not None and len(charuco_corners) > 0 else 0

    success = num_corners >= 6
    return success, charuco_corners, charuco_ids, num_markers, num_corners


def calibrate_with_checkerboard(image_folder, cols, rows, square_size_mm, camera_name):
    print(f"\n{'='*60}")
    print(f"  CALIBRATING: {camera_name}")
    print(f"  Board: Standard Checkerboard {cols}x{rows} inner corners")
    print(f"  Square size: {square_size_mm} mm")
    print(f"{'='*60}\n")

    # Tricky part: objp holds the real-world 3D position of every inner corner, with
    # Z=0 because the board is flat. mgrid lays them on a grid, then we scale to mm.
    # The same objp is reused for every photo (the board does not change shape).
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_mm

    obj_points_list = []
    img_points_list = []

    extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
    image_files = []
    for ext in extensions:
        image_files.extend(glob.glob(os.path.join(image_folder, ext)))
    image_files.sort()

    if len(image_files) == 0:
        print(f"ERROR: No images found in {image_folder}")
        return None

    print(f"Found {len(image_files)} images\n")

    image_size = None
    good_images = 0
    bad_images = 0

    for filepath in image_files:
        filename = os.path.basename(filepath)
        img = cv2.imread(filepath)
        if img is None:
            print(f"  [SKIP] {filename} - could not read file")
            bad_images += 1
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray.shape[::-1]
            print(f"Image resolution: {image_size[0]}x{image_size[1]}\n")

        found, corners = detect_checkerboard(gray, cols, rows)

        if found:
            obj_points_list.append(objp)
            img_points_list.append(corners)
            good_images += 1
            print(f"  [OK]   {filename} - found {len(corners)} corners")
        else:
            bad_images += 1
            print(f"  [FAIL] {filename} - checkerboard not detected")

    print(f"\n--- Detection Summary ---")
    print(f"Good images: {good_images}/{len(image_files)}")
    print(f"Failed:      {bad_images}/{len(image_files)}")

    if good_images < 5:
        print(f"\nERROR: Need at least 5 good images, got {good_images}")
        print("Tips: Make sure the board is fully visible and the image is sharp.")
        return None

    if good_images < 10:
        print(f"\nWARNING: Only {good_images} good images. 15-20 is recommended.")

    print(f"\nRunning calibration with {good_images} images...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points_list, img_points_list, image_size, None, None
    )

    return {
        'camera_matrix': camera_matrix,
        'dist_coeffs': dist_coeffs,
        'image_size': image_size,
        'rms_error': ret,
        'num_images': good_images,
        'rvecs': rvecs,
        'tvecs': tvecs,
    }


def calibrate_with_charuco(image_folder, board_cols, board_rows, square_size_mm,
                           marker_size_mm, dict_id, camera_name):
    print(f"\n{'='*60}")
    print(f"  CALIBRATING: {camera_name}")
    print(f"  Board: ChArUco {board_cols}x{board_rows}")
    print(f"  Square: {square_size_mm}mm, Marker: {marker_size_mm}mm")
    print(f"{'='*60}\n")

    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    board = cv2.aruco.CharucoBoard(
        (board_cols, board_rows), square_size_mm, marker_size_mm, aruco_dict
    )

    det_params = cv2.aruco.DetectorParameters()
    det_params.adaptiveThreshWinSizeMin = 3
    det_params.adaptiveThreshWinSizeMax = 73
    det_params.adaptiveThreshWinSizeStep = 4
    det_params.minMarkerPerimeterRate = 0.005
    det_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    charuco_params = cv2.aruco.CharucoParameters()
    charuco_params.tryRefineMarkers = True

    charuco_detector = cv2.aruco.CharucoDetector(board, charuco_params, det_params)

    extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
    image_files = []
    for ext in extensions:
        image_files.extend(glob.glob(os.path.join(image_folder, ext)))
    image_files.sort()

    if len(image_files) == 0:
        print(f"ERROR: No images found in {image_folder}")
        return None

    print(f"Found {len(image_files)} images\n")

    image_size = None
    all_charuco_corners = []
    all_charuco_ids = []
    good_images = 0
    bad_images = 0

    expected_corners = (board_cols - 1) * (board_rows - 1)

    for filepath in image_files:
        filename = os.path.basename(filepath)
        img = cv2.imread(filepath)
        if img is None:
            print(f"  [SKIP] {filename} - could not read file")
            bad_images += 1
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray.shape[::-1]
            print(f"Image resolution: {image_size[0]}x{image_size[1]}\n")

        success, corners, ids, n_markers, n_corners = detect_charuco(
            gray, board, charuco_detector
        )

        if success:
            all_charuco_corners.append(corners)
            all_charuco_ids.append(ids)
            good_images += 1
            print(f"  [OK]   {filename} - {n_markers} markers, {n_corners}/{expected_corners} corners")
        else:
            bad_images += 1
            print(f"  [FAIL] {filename} - {n_markers} markers, {n_corners} corners (need >=6)")

    print(f"\n--- Detection Summary ---")
    print(f"Good images: {good_images}/{len(image_files)}")
    print(f"Failed:      {bad_images}/{len(image_files)}")

    if good_images < 5:
        print(f"\nERROR: Need at least 5 good images, got {good_images}")
        return None

    print(f"\nRunning calibration with {good_images} images...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        all_charuco_corners, all_charuco_ids, board, image_size, None, None
    )

    return {
        'camera_matrix': camera_matrix,
        'dist_coeffs': dist_coeffs,
        'image_size': image_size,
        'rms_error': ret,
        'num_images': good_images,
        'rvecs': rvecs,
        'tvecs': tvecs,
    }


def print_results(result, camera_name):
    if result is None:
        print(f"\nCalibration FAILED for {camera_name}!")
        return

    print(f"\n{'='*60}")
    print(f"  CALIBRATION RESULTS: {camera_name}")
    print(f"{'='*60}")

    rms = result['rms_error']
    if rms < 0.5:
        quality = "EXCELLENT"
    elif rms < 1.0:
        quality = "GOOD"
    elif rms < 2.0:
        quality = "ACCEPTABLE"
    else:
        quality = "POOR - retake photos!"

    print(f"\n  RMS Reprojection Error: {rms:.4f} pixels ({quality})")
    print(f"  Images used: {result['num_images']}")
    print(f"  Image size:  {result['image_size'][0]}x{result['image_size'][1]}")

    fx = result['camera_matrix'][0, 0]
    fy = result['camera_matrix'][1, 1]
    cx = result['camera_matrix'][0, 2]
    cy = result['camera_matrix'][1, 2]
    print(f"\n  Camera Matrix:")
    print(f"    Focal length: fx={fx:.1f}, fy={fy:.1f} pixels")
    print(f"    Optical center: cx={cx:.1f}, cy={cy:.1f} pixels")

    d = result['dist_coeffs'].flatten()
    print(f"\n  Distortion Coefficients:")
    print(f"    k1={d[0]:.6f} (radial)")
    print(f"    k2={d[1]:.6f} (radial)")
    print(f"    p1={d[2]:.6f} (tangential)")
    print(f"    p2={d[3]:.6f} (tangential)")
    if len(d) > 4:
        print(f"    k3={d[4]:.6f} (radial)")


def save_calibration(result, camera_name, output_folder="."):
    if result is None:
        return

    output_path = os.path.join(output_folder, f"calibration_{camera_name}.npz")

    np.savez(
        output_path,
        camera_matrix=result['camera_matrix'],
        dist_coeffs=result['dist_coeffs'],
        image_size=np.array(result['image_size']),
        rms_error=np.array([result['rms_error']]),
    )
    print(f"\n  Saved calibration to: {output_path}")

    json_path = os.path.join(output_folder, f"calibration_{camera_name}.json")
    json_data = {
        'camera_name': camera_name,
        'image_size': list(result['image_size']),
        'rms_error': float(result['rms_error']),
        'num_images_used': result['num_images'],
        'camera_matrix': result['camera_matrix'].tolist(),
        'dist_coeffs': result['dist_coeffs'].tolist(),
    }
    with open(json_path, 'w') as f:
        json.dump(json_data, f, indent=2)
    print(f"  Saved readable copy to: {json_path}")


def test_undistortion(result, image_folder, camera_name, output_folder="."):
    if result is None:
        return

    extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
    image_files = []
    for ext in extensions:
        image_files.extend(glob.glob(os.path.join(image_folder, ext)))
    image_files.sort()

    if not image_files:
        return

    img = cv2.imread(image_files[0])
    h, w = img.shape[:2]

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        result['camera_matrix'], result['dist_coeffs'], (w, h), alpha=1, newImgSize=(w, h)
    )

    undistorted = cv2.undistort(
        img, result['camera_matrix'], result['dist_coeffs'], None, new_camera_matrix
    )

    comparison = np.hstack([img, undistorted])
    comp_path = os.path.join(output_folder, f"comparison_{camera_name}.jpg")
    cv2.imwrite(comp_path, comparison)
    print(f"  Saved before/after comparison to: {comp_path}")

    undist_path = os.path.join(output_folder, f"undistorted_{camera_name}.jpg")
    cv2.imwrite(undist_path, undistorted)
    print(f"  Saved undistorted image to: {undist_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Camera Calibration for Crane Vision System",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        '--folder', type=str, required=True,
        help='Folder containing calibration images'
    )
    parser.add_argument(
        '--camera', type=str, default='cam1',
        help='Camera name (e.g. cam1, cam2). Used for output filenames.'
    )
    parser.add_argument(
        '--board', type=str, default='checkerboard', choices=['checkerboard', 'charuco'],
        help='Type of calibration board:\n'
             '  checkerboard = standard black/white checkerboard (recommended)\n'
             '  charuco = ChArUco board generated by OpenCV'
    )
    parser.add_argument(
        '--square-size', type=float, default=None,
        help='Actual measured square size in mm (MEASURE WITH RULER!)'
    )
    parser.add_argument(
        '--output', type=str, default='.',
        help='Output folder for calibration files'
    )

    args = parser.parse_args()

    square_size = args.square_size if args.square_size else SQUARE_SIZE_MM
    if args.square_size is None:
        print("WARNING: You did not provide --square-size.")
        print(f"Using default {SQUARE_SIZE_MM}mm. MEASURE YOUR PRINTED BOARD WITH A RULER!")
        print("")

    os.makedirs(args.output, exist_ok=True)

    if args.board == 'checkerboard':
        result = calibrate_with_checkerboard(
            args.folder, CHECKERBOARD_COLS, CHECKERBOARD_ROWS,
            square_size, args.camera
        )
    else:
        result = calibrate_with_charuco(
            args.folder, CHARUCO_COLS, CHARUCO_ROWS,
            square_size, MARKER_SIZE_MM, CHARUCO_DICT, args.camera
        )

    print_results(result, args.camera)
    save_calibration(result, args.camera, args.output)
    test_undistortion(result, args.folder, args.camera, args.output)

    print(f"\n{'='*60}")
    print(f"  DONE!")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
