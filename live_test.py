import cv2
from ultralytics import YOLO
import cv2.aruco as aruco

print("⏳ Loading Custom Trained YOLO Model...")
model = YOLO("yolo_model.pt")

print("📷 Opening Camera...")
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters()

print("✅ Live Stream Started! Press 'q' on your keyboard to close the window.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame.")
        break

    results = model(frame, conf=0.45, verbose=False)
    annotated_frame = results[0].plot()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(annotated_frame, corners, ids)

    display_frame = cv2.resize(annotated_frame, (1280, 720))
    cv2.imshow("Crane Live Vision Test (Press 'q' to quit)", display_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
