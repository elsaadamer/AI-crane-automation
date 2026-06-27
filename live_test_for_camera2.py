import cv2
import cv2.aruco as aruco
import numpy as np

MARKER_SIZE_CM = 6.8
FRAME_W = 1920
FRAME_H = 1080
FOCAL_LENGTH = 1200.0

camera_matrix = np.array([
    [FOCAL_LENGTH, 0, FRAME_W / 2],
    [0, FOCAL_LENGTH, FRAME_H / 2],
    [0, 0, 1]
], dtype=float)
dist_coeffs = np.zeros((4, 1))

obj_points = np.array([
    [-MARKER_SIZE_CM / 2,  MARKER_SIZE_CM / 2, 0],
    [ MARKER_SIZE_CM / 2,  MARKER_SIZE_CM / 2, 0],
    [ MARKER_SIZE_CM / 2, -MARKER_SIZE_CM / 2, 0],
    [-MARKER_SIZE_CM / 2, -MARKER_SIZE_CM / 2, 0]
], dtype=np.float32)

def adjust_gamma(image, gamma=0.5):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

def get_transform_matrix(rvec, tvec):
    rmat, _ = cv2.Rodrigues(rvec)
    transform_matrix = np.eye(4)
    transform_matrix[0:3, 0:3] = rmat
    transform_matrix[0:3, 3] = tvec.reshape(3)
    return transform_matrix

def main():
    print("📷 Opening Camera 2...")
    cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print("❌ Error: Could not open camera at index 2.")
        return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    target_ids = [0, 1, 2, 10]

    print("✅ Camera active! Origin is locked to ID 10.")
    print("👉 Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = adjust_gamma(gray, gamma=0.5)

        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(aruco_dict, aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            T_cam_to_id10 = None

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == 10:
                    success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, dist_coeffs)
                    if success:
                        T_cam_to_id10 = get_transform_matrix(rvec, tvec)
                    break

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in target_ids:
                    success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, dist_coeffs)

                    if success:
                        cX = int(np.mean(corners[i][0][:, 0]))
                        cY = int(np.mean(corners[i][0][:, 1]))

                        if T_cam_to_id10 is not None:
                            T_cam_to_target = get_transform_matrix(rvec, tvec)

                            T_id10_to_cam = np.linalg.inv(T_cam_to_id10)
                            T_id10_to_target = np.dot(T_id10_to_cam, T_cam_to_target)

                            rel_x = T_id10_to_target[0, 3]
                            rel_y = T_id10_to_target[1, 3]
                            rel_z = T_id10_to_target[2, 3]

                            if marker_id == 10:
                                coord_text = f"ID:10 (ORIGIN) 0,0,0"
                            else:
                                coord_text = f"ID:{marker_id} X:{rel_x:.1f} Y:{rel_y:.1f} Z:{rel_z:.1f}cm"

                        else:
                            coord_text = f"ID:{marker_id} X:{tvec[0][0]:.1f} Y:{tvec[1][0]:.1f} Z:{tvec[2][0]:.1f}cm (Cam)"

                        cv2.rectangle(frame, (cX - 45, cY - 35), (cX + 300, cY - 10), (0, 0, 0), -1)
                        cv2.putText(frame, coord_text, (cX - 40, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE_CM / 2)

        display_frame = cv2.resize(frame, (1280, 720))
        cv2.imshow("Camera 2 - Local Coordinates (ID 10)", display_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
