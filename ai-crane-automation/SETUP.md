# Setup and Technical Guide

Full instructions to install the software and run the crane from your own laptop. For the project overview, see [README.md](README.md).

## Project files: what each script does

All Python files live in **one flat folder**, on purpose. `server.py` imports `controller`, `detector`, and `navigator` by name, `detector.py` imports `use_calibration`, and the data files are loaded by plain file name. Everything must stay together.

**The four files that run the crane**

- **`server.py`** is the one you launch. It is the MCP server. It creates the controller, detector, and navigator and exposes four tools (`scan_environment`, `move_crane`, `operate_hook`, `toggle_magnet`) to the language model.
- **`detector.py`** is the eyes. It opens the camera, removes lens distortion, runs YOLOv8 and ArUco, and converts each point from pixels to centimeters with the homography.
- **`navigator.py`** is the brain for movement. It tracks the crane position, knows the rail limits and speeds, checks paths against obstacles, picks a safe route, and sends timed motor commands.
- **`controller.py`** is the hands. The only file that talks to the hardware, over USB serial to the Arduino Uno.

A "move to Station B" flows like this: the LLM calls a tool in `server.py`, which asks `detector.py` what it sees, passes the obstacles to `navigator.py` to plan a safe path, and `navigator.py` sends the moves through `controller.py` to the Arduino.

**Calibration helpers (run once per camera setup)**

- **`use_calibration.py`** loads the lens calibration and undistorts frames. Shared by `detector.py` and the homography script.
- **`calibrate_camera.py`** produces `calibration_cam1.npz` from checkerboard photos.
- **`calibrate_homography.py`** produces `homography_cam1.npy` from four ArUco markers.

**Viewing tools**

- **`live_test.py`** shows the overhead camera with YOLO and ArUco overlays.
- **`live_test_for_camera2.py`** shows the side camera with marker positions in centimeters.

**Arduino firmware**

- **`arduino/for_crane_project/for_crane_project.ino`** runs on the Uno: joystick, motors, serial parsing, and magnet relay to the Nano.
- **`arduino/for_nano_crane_magnet/for_nano_crane_magnet.ino`** runs on the Nano: switches the electromagnet, kept separate so it holds even if the Uno resets.

**Data files (required)**

- **`yolo_model.pt`** trained YOLOv8 model, six classes.
- **`calibration_cam1.npz`** lens calibration (`calibration_cam1.json` is the readable copy).
- **`homography_cam1.npy`** pixel-to-centimeter matrix.

## Hardware

| Part | Role |
|------|------|
| Arduino Uno | Main controller. Motors via L298N, joystick, serial at 115200 baud, talks to the Nano. |
| Arduino Nano | Electromagnet control over SoftwareSerial at 9600 baud. Holds even if the Uno reboots. |
| 3x DC motors + L298N | Body rotation, trolley, hook. |
| Electromagnet | QUARKZMAN 5V 50N in a 3D printed housing. |
| Camera 1 (overhead) | Elgato Facecam Neo. Main camera for YOLO, ArUco, homography. |
| Analog joystick | Manual control (X = trolley, Y = hook, Z = body). |

Wiring is in `docs/schematic.png`. Uno A5 (TX) to Nano D10 (RX), common ground. Magnet protocol is one character: `'1'` on, `'0'` off.

## Software setup, command by command

Tested on Windows with Python 3.11.

1. Install **Python 3.11** from python.org and tick "Add python.exe to PATH".
2. Open Command Prompt in the folder that contains `server.py`:

```bat
cd C:\Users\YOUR_NAME\Desktop\ai-crane-automation
```

3. Type these one by one:

```bat
python -m venv venv
venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

After the activate line your prompt starts with `(venv)`. In PowerShell use `venv\Scripts\Activate.ps1`.

4. Check the key libraries load:

```bat
python -c "import cv2.aruco; print('aruco ok')"
python -c "from ultralytics import YOLO; YOLO('yolo_model.pt'); print('yolo ok')"
```

Notes: use **opencv-contrib-python**, not plain `opencv-python` (the code needs `cv2.aruco`); do not install both. `ultralytics` brings YOLOv8 and PyTorch (CPU). `pyserial` is imported as `serial`.

## Connecting to Claude (the LLM brain)

1. Install **Claude Desktop** from [claude.ai/download](https://claude.ai/download) and sign in.
2. In Claude Desktop: **Settings > Developer > Edit Config**. This opens `claude_desktop_config.json` (Windows: `%APPDATA%\Claude\`, macOS: `~/Library/Application Support/Claude/`).
3. Add the server, using absolute paths and the venv Python:

```json
{
  "mcpServers": {
    "crane": {
      "command": "C:\\Users\\YOUR_NAME\\Desktop\\ai-crane-automation\\venv\\Scripts\\python.exe",
      "args": ["C:\\Users\\YOUR_NAME\\Desktop\\ai-crane-automation\\server.py"]
    }
  }
}
```

4. Quit Claude Desktop fully and reopen it. Config loads only on a fresh start.
5. In a new chat the crane's tools appear. Try: "Scan the table and tell me what you see," then "Pick up the screw and move it to Station B."

Logs for troubleshooting: `%APPDATA%\Claude\logs` (look for `mcp-server-crane.log`). To test alone, run `python server.py` inside the venv.

## Camera calibration and homography

Camera 1's files are included, so you only redo this if you change the camera or table.

- **Lens calibration:** `python calibrate_camera.py --folder path\to\photos --camera cam1 --square-size 22`. Produces `calibration_cam1.npz` (about 1.2 px RMS over 95 images for this camera).
- **Homography:** `python calibrate_homography.py --camera-index 1`. Place markers 0, 1, 2, 3, press SPACE when all four are seen. Produces `homography_cam1.npy` (maps the reference markers back to within about half a centimeter).

## Serial command protocol

| Command | Meaning |
|---------|---------|
| `<J,body,hook,trolley,duration_ms>` | Drive the motors for a time in milliseconds. |
| `<M,1>` / `<M,0>` | Magnet on / off. |
| `<S>` | Emergency stop. |

Replies: `<Ready>`, `<ACK>`, `<DONE>`, `<MAGNET_ON>`, `<MAGNET_OFF>`, `<STOPPED>`. Timing uses `millis()`, so the board keeps reading commands while moving.

## Known limits and things to edit first

- **COM port is hardcoded** in `controller.py` (`COM10`). Change it.
- **Camera index** is assumed (`detector.py` uses 0, `server.py` tries 1 then 0). Change if needed.
- **The fence is not treated as an obstacle:** `DANGER_OBJECTS` lists `"Fence"` (capital F) but the check lowercases names, so it never matches. One character fix, left as is to match the thesis state.
- **Movement is open loop and time based.** No encoder feedback, so position drifts over many moves. A main reason for the stepper motor redesign.
- **The path planner tries only two L-shaped routes.** In tight clusters it can stop even when a longer route exists.

## License

Part of an ongoing Master's thesis. All rights reserved until submission; please ask before reusing. A formal open license will follow.
