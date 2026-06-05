from mcp.server.fastmcp import FastMCP
import sys
import os
import cv2
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
os.chdir(current_dir)

from controller import CraneController
from detector import get_detector, CraneDetector
from navigator import get_navigator

mcp = FastMCP("Construction Crane V2 + VLM")
_controller = CraneController()
_global_detector = None

DANGER_OBJECTS = ["worker", "construction-vehicle", "Fence", "wheelbarrow"]


def get_fresh_detector():
    global _global_detector
    if _global_detector is None:
        _global_detector = CraneDetector()

    if _global_detector.cap is None or not _global_detector.cap.isOpened():
        sys.stderr.write("DEBUG: Opening Camera...\n")
        _global_detector.cap = cv2.VideoCapture(1)
        if not _global_detector.cap.isOpened():
            _global_detector.cap = cv2.VideoCapture(0)

        _global_detector.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        _global_detector.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    return _global_detector

@mcp.tool()
def scan_environment(mode: str = "all") -> str:
    """
    Step 1: Scans table using YOLO for precise coordinates (X, Y).
    Returns list of objects found.
    """
    detector = None
    try:
        sys.stderr.write(f"DEBUG: YOLO Scan requested ({mode})...\n")
        detector = get_fresh_detector()
        detector.cap.read()

        objects, scale = detector.scan_scene(mode)

        msg = "REPORT:\n"
        if scale: msg += "✅ CALIBRATION: Valid.\n"

        msg += f"👁️ YOLO DETECTED ({len(objects)} items):\n"
        for obj in objects:
            is_danger = obj['name'].lower() in DANGER_OBJECTS
            tag = "⛔ [OBSTACLE]" if is_danger else ""
            msg += f" • {obj['name'].upper():<15} -> X={obj['x']:>5.1f}, Y={obj['y']:>5.1f} {tag}\n"
        return msg
    except Exception as e:
        return f"Error: {e}"


@mcp.tool()
def move_crane(x: float, y: float) -> str:
    """
    Step 3: Moves the crane to X, Y (cm).
    AUTOMATICALLY SCANS & DETOURS AROUND OBSTACLES.
    """
    try:
        sys.stderr.write(f"DEBUG: Smart Move to ({x}, {y})...\n")

        detector = get_fresh_detector()
        detector.cap.read()
        objects, _ = detector.scan_scene("all")
        obstacles = [obj for obj in objects if obj['name'].lower() in DANGER_OBJECTS]

        if obstacles:
            sys.stderr.write(f"⚠️  Safety System: Found {len(obstacles)} potential obstacles.\n")

        if not _controller.ser: _controller.connect()
        nav = get_navigator(_controller)
        result = nav.move_to(x, y, obstacles=obstacles)
        return result

    except Exception as e:
        return f"Move Error: {str(e)}"
@mcp.tool()
def operate_hook(action: str, duration: float = 2.0) -> str:
    """
    Moves the hook/magnet up or down.
    action: "up" or "down"
    duration: Time in seconds to move (default 2.0s)
    """
    if not _controller.ser: _controller.connect()
    nav = get_navigator(_controller)
    return nav.move_hook(action, duration)

@mcp.tool()
def toggle_magnet(state: str) -> str:
    """
    Step 4: Turns the electromagnet ON or OFF.
    state: "on" to activate (take object), "off" to deactivate (drop object).
    """
    try:
        if not _controller.ser:
            _controller.connect()

        is_on = True if state.lower() == "on" else False

        success = _controller.set_magnet(is_on)

        if success:
            return f"Magnet successfully turned {state.upper()}."
        else:
            return "Failed to communicate with the crane controller."
    except Exception as e:
        return f"Magnet Error: {str(e)}"


if __name__ == "__main__":
    print(" Crane Server Running")
    mcp.run()
