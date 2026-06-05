import cv2
import numpy as np
from ultralytics import YOLO
import cv2.aruco as aruco
from use_calibration import load_calibration, create_undistort_maps, undistort_frame


class CraneDetector:
    def __init__(self, model_path="yolo_model.pt", camera_index=0):
        try:
            self.model = YOLO(model_path)
        except:
            self.model = None

        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()

        cam1_matrix, cam1_dist, cam1_size = load_calibration("calibration_cam1.npz")
        self.camera_matrix = cam1_matrix
        self.dist_coeffs = cam1_dist

        self.undist_map1, self.undist_map2, _ = create_undistort_maps(
            cam1_matrix, cam1_dist, cam1_size, crop=True
        )

        self.H = np.load("homography_cam1.npy")

        self.last_scale = 1.0

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return undistort_frame(frame, self.undist_map1, self.undist_map2)

    def detect_markers_robust(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        return corners, ids

    def pixel_to_cm(self, u, v):
        pt_in = np.array([[[float(u), float(v)]]], dtype=np.float32)
        pt_out = cv2.perspectiveTransform(pt_in, self.H)
        return float(pt_out[0, 0, 0]), float(pt_out[0, 0, 1])

    def scan_scene(self, target_name="all"):
        frame = self.get_frame()
        if frame is None:
            return [], None

        corners, ids = self.detect_markers_robust(frame)

        found_objects = []

        if ids is not None:
            ids_flat = ids.flatten()
            for i, marker_id in enumerate(ids_flat):
                c = corners[i][0]
                cx_px = float((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4.0)
                cy_px = float((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4.0)

                if marker_id == 1:
                    x_cm, y_cm = self.pixel_to_cm(cx_px, cy_px)
                    found_objects.append({"name": "STATION_A", "x": x_cm, "y": y_cm})
                elif marker_id == 2:
                    x_cm, y_cm = self.pixel_to_cm(cx_px, cy_px)
                    found_objects.append({"name": "STATION_B", "x": x_cm, "y": y_cm})

        if self.model:
            results = self.model(frame, verbose=False, imgsz=1280)
            for result in results:
                for box in result.boxes:
                    if float(box.conf[0]) > 0.45:
                        cls_id = int(box.cls[0])
                        name = self.model.names[cls_id]
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx_px = float((x1 + x2) / 2.0)
                        cy_px = float((y1 + y2) / 2.0)

                        x_cm, y_cm = self.pixel_to_cm(cx_px, cy_px)
                        found_objects.append({"name": name, "x": x_cm, "y": y_cm})

        return found_objects, self.last_scale


_detector = None
def get_detector():
    global _detector
    if _detector is None:
        _detector = CraneDetector()
    return _detector
