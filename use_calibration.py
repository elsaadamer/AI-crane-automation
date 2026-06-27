import cv2
import numpy as np


def load_calibration(calibration_file):
    data = np.load(calibration_file)
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']
    image_size = tuple(data['image_size'])
    rms = data['rms_error'][0]

    print(f"Loaded calibration from {calibration_file}")
    print(f"  Image size: {image_size[0]}x{image_size[1]}")
    print(f"  RMS error: {rms:.4f}")

    return camera_matrix, dist_coeffs, image_size


def create_undistort_maps(camera_matrix, dist_coeffs, image_size, crop=True):
    w, h = image_size
    alpha = 0 if crop else 1

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), alpha, newImgSize=(w, h)
    )

    # Tricky part: build the pixel remap table only once here. Doing this every frame
    # would be slow, so detector.py reuses these two maps with cv2.remap instead.
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_16SC2
    )

    return map1, map2, new_camera_matrix


def undistort_frame(frame, map1, map2):
    return cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 3:
        print("Usage: python use_calibration.py calibration_cam1.npz test_image.jpg")
        sys.exit(1)

    calib_file = sys.argv[1]
    test_image = sys.argv[2]

    cam_matrix, dist_coeffs, img_size = load_calibration(calib_file)
    map1, map2, new_matrix = create_undistort_maps(cam_matrix, dist_coeffs, img_size)

    img = cv2.imread(test_image)
    undistorted = undistort_frame(img, map1, map2)

    cv2.imwrite("undistorted_test.jpg", undistorted)
    print("Saved: undistorted_test.jpg")
