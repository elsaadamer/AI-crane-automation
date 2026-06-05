# AI-Controlled Tower Crane Automation

A physical model tower crane that a Large Language Model (LLM) can drive on its own using computer vision. A camera looks down at the table, finds the stations and the obstacles, and turns what it sees into real centimeters. The LLM then decides where to move. A safety layer sits in the middle and refuses any path that would cross a worker or another obstacle.

This is the practical part of my Master's thesis at TU Clausthal, *"AI-Controlled Tower Crane Automation: Integrating Computer Vision and Large Language Model Control on a Physical Model Crane"* (Chair of Intelligent Automation Systems, supervised by Prof. Dr. Stefan Palis).

> **Version note.** This repository is the **DC motor prototype** (version 1). It uses three DC motors driven by L298N H-bridges. After this version, a decision was made to redesign the crane: the mechanical structure is being rebuilt to be more stable, and the three DC motors are being replaced with NEMA 17 stepper motors and TB6600 drivers for more accurate, repeatable movement. The redesigned stepper version will be uploaded as a separate release as soon as it is finished and tested.

---

## Table of contents

1. [What the system does](#what-the-system-does)
2. [How it works, step by step](#how-it-works-step-by-step)
3. [Hardware and electronics](#hardware-and-electronics)
4. [Repository structure](#repository-structure)
5. [Software setup](#software-setup)
6. [Camera calibration and homography](#camera-calibration-and-homography)
7. [Live test tools](#live-test-tools)
8. [Running the full system](#running-the-full-system)
9. [Serial command protocol](#serial-command-protocol)
10. [Known limits and things to edit first](#known-limits-and-things-to-edit-first)
11. [License and academic use](#license-and-academic-use)

---

## What the system does

You give the crane a task in plain language, for example "pick up the screw and move it to Station B." The system then:

1. Takes a picture of the work area with an overhead camera.
2. Finds the stations (paper markers) and the objects (a worker figure, a vehicle, a wheelbarrow, a screw).
3. Converts every pixel position into real centimeters on the table.
4. Lets the LLM choose a target and call the crane functions.
5. Checks the planned path. If it would pass too close to a worker or obstacle, the move is blocked and the LLM is told to find another route.
6. Drives the motors, raises or lowers the hook, and switches the electromagnet on or off.

The LLM never controls the motors directly. It can only call a small set of safe functions (scan, move, hook, magnet). This keeps the safety rules in normal code, not inside the language model.

---

## How it works, step by step

The vision and control pipeline has four parts. Here is what each one does in simple terms.

### 1. YOLOv8: finding the objects

YOLO ("You Only Look Once") is an object detection model. You give it an image and it returns boxes around the things it recognizes, with a class name and a confidence score from 0 to 1.

In this project YOLO is **not trained from scratch**. A custom model (`yolo_model.pt`) was trained earlier on photos of the model construction site, and here it is only used for detection. It knows six classes, including the worker and the obstacle types. The code keeps a detection only if its confidence is above 0.45, which removes most false alarms. For each kept box, the center pixel is taken as the object position.

### 2. ArUco markers: finding the stations and the origin

ArUco markers are small black and white squares, like simple QR codes. Each one has an ID number. They are easier and more exact to locate than a YOLO box, so they are used for the fixed points:

- **ID 0** is the origin, the point that means (0, 0).
- **ID 1** is Station A.
- **ID 2** is Station B.

The code uses the dictionary `DICT_4X4_50`. The marker center is the average of its four corners.

### 3. Homography: turning pixels into centimeters

The camera looks at the table from an angle, so the table looks like a trapezoid in the image, not a clean rectangle. A simple "multiply pixels by a scale factor" would be wrong, because distances near the camera and far from it are not the same.

A **homography** fixes this. It is a 3x3 matrix that maps a point in the image plane to a point on the flat table plane. Once you measure a few known points with a ruler, you can compute this matrix once and reuse it. After that, any pixel `(u, v)` is turned into table coordinates `(x_cm, y_cm)` with one matrix multiplication (`cv2.perspectiveTransform`). The lens distortion is removed first (see the calibration section), then the homography is applied.

### 4. MCP and the LLM: the brain

[MCP (Model Context Protocol)](https://modelcontextprotocol.io) is the bridge between the Python code and the language model. `server.py` exposes four tools to the LLM:

- `scan_environment` returns the list of detected objects with their centimeter coordinates.
- `move_crane(x, y)` rescans for obstacles, then moves the crane to a target while avoiding them.
- `operate_hook(action, duration)` raises or lowers the hook.
- `toggle_magnet(state)` turns the electromagnet on or off.

The LLM reads the scan, reasons about the layout, and calls these tools in order. The text written under each tool in `server.py` is what the model reads to understand the tool, so those descriptions are part of the program, not just comments.

### The safety layer (the most important part)

Path checking lives in `navigator.py`, not in the LLM. When a move is requested, the navigator tries two L-shaped paths to the target: "move Y first, then X" and "move X first, then Y." For each path it measures the shortest distance from every obstacle to the path line. If any obstacle is closer than `OBSTACLE_RADIUS + SAFETY_MARGIN`, that path is rejected. If both paths are blocked, the crane stops and reports which object blocked it. This is why the crane will not drive through the worker even if you ask it to.

---

## Hardware and electronics

| Part | Role |
|------|------|
| Arduino Uno | Main controller. Reads serial commands, drives the three DC motors through L298N boards, runs the joystick, talks to the Nano. Serial at 115200 baud. |
| Arduino Nano | Controls the electromagnet through its own L298N. Listens to the Uno over SoftwareSerial at 9600 baud. Keeps the magnet holding even if the Uno reboots. |
| 3x DC motors + L298N | Body rotation, trolley, and hook. |
| Electromagnet | QUARKZMAN 5V DC 50N, 25x20 mm, in a 3D printed housing. Picks up steel parts. |
| Camera 1 (overhead) | Elgato Facecam Neo, top view. Used for YOLO and the homography map. |
| Camera 2 (table level) | Used for the side view test. |
| Analog joystick | Manual control when the AI is not driving (X = trolley, Y = hook, Z = body). |

### Wiring

The full wiring of the control box is in the schematic below. It shows the Arduino Uno on a sensor shield, the three L298N drivers (Trolley, Hook, Body), the Arduino Nano for the magnet, the joystick, and the external power supply and PC connections.

![Electronics schematic](docs/schematic.png)

Key points:

- Uno talks to the Nano over SoftwareSerial: **Uno A5 (TX) to Nano D10 (RX)**, with a common ground between both boards.
- The magnet protocol between Uno and Nano is a single character: `'1'` for on, `'0'` for off.
- Pin assignments are at the top of `arduino/for_crane_project/for_crane_project.ino`.

> The stepper version mentioned at the top of this README will replace the L298N drivers with TB6600 drivers and the DC motors with NEMA 17 steppers. That schematic will come with the next release.

---

## Repository structure

```
ai-crane-automation/
  README.md
  requirements.txt
  .gitignore

  controller.py              Serial link to the Arduino Uno
  detector.py                Camera + YOLO + ArUco + homography
  navigator.py               Path planning and obstacle avoidance
  server.py                  MCP server, exposes the tools to the LLM

  live_test.py               Overhead camera viewer (YOLO + ArUco overlay)
  live_test_for_camera2.py   Side camera viewer (ArUco pose relative to ID 10)

  calibrate_camera.py        Lens calibration from checkerboard photos
  calibrate_homography.py    Builds the pixel to cm matrix from 4 markers
  use_calibration.py         Helper functions used by detector.py

  calibration_cam1.npz       Camera 1 lens calibration (included)
  calibration_cam1.json      Same data in readable form (included)
  homography_cam1.npy        Camera 1 pixel to cm matrix (GENERATED, see below)
  yolo_model.pt              Trained YOLOv8 weights (6 classes)

  arduino/
    for_crane_project/
      for_crane_project.ino           Arduino Uno sketch
    for_nano_crane_magnet/
      for_nano_crane_magnet.ino       Arduino Nano sketch

  docs/
    schematic.png                     Control box wiring
    comparison_cam1.jpg               Before/after lens undistortion, Camera 1
    comparison_cam2.jpg               Before/after lens undistortion, Camera 2
    undistorted_cam1.jpg              Undistorted sample, Camera 1
    undistorted_cam2.jpg              Undistorted sample, Camera 2
    homography_cam1_reference.jpg     Frame used to build the homography
```

Each `.ino` file sits inside a folder with the same name. The Arduino IDE requires this, so open the folder, not the file directly.

---

## Software setup

Tested on Windows with Python 3.11. A virtual environment ("venv") is a private package folder for one project. It keeps these libraries separate so they do not break your other Python projects.

**1. Install Python 3.11** from [python.org](https://www.python.org/downloads/). During install, tick **"Add python.exe to PATH"**. Python 3.11 is the safest choice for `ultralytics` and PyTorch at the moment.

**2. Open a terminal in the project folder.**

```bat
cd path\to\ai-crane-automation
```

**3. Create the virtual environment.**

```bat
python -m venv venv
```

**4. Activate it.**

```bat
venv\Scripts\activate
```

In PowerShell use `venv\Scripts\Activate.ps1` instead. When it is active you will see `(venv)` at the start of the line.

**5. Update pip.**

```bat
python -m pip install --upgrade pip
```

**6. Install the packages.**

```bat
pip install -r requirements.txt
```

`requirements.txt` contains:

```
ultralytics
opencv-contrib-python
numpy
pyserial
mcp
```

Three points that often cause trouble:

- `ultralytics` installs YOLOv8 and pulls in PyTorch for you (the CPU version). YOLOv8 runs fine on a CPU, just slower than on a GPU.
- Use **opencv-contrib-python**, not plain `opencv-python`. This project uses `cv2.aruco`, which is only in the contrib build. Never install both, because they conflict. If `ultralytics` already installed plain `opencv-python`, remove it first: `pip uninstall opencv-python`, then `pip install opencv-contrib-python`.
- `pyserial` installs under that name but you import it in Python as `serial`. This is normal, not an error.

**7. Check that the key libraries load.**

```bat
python -c "import cv2.aruco; print('aruco ok')"
python -c "from ultralytics import YOLO; YOLO('yolo_model.pt'); print('yolo ok')"
```

Run the second command from the folder that holds `yolo_model.pt`. Both should print "ok" with no error.

**8. When you are done, leave the environment.**

```bat
deactivate
```

---

## Camera calibration and homography

The detector needs two things to turn what the camera sees into accurate centimeters. Both are specific to one physical camera, lens, and table layout, so you generate your own if you change the setup. Camera 1's files are already included here.

### Step 1: Lens calibration (removes the curved-line distortion)

A camera lens bends straight lines, more so near the edges of the frame. Lens calibration measures this bending and lets us undo it.

`calibrate_camera.py` does this from photos of a printed checkerboard. You take 15 to 20 photos of the board from different angles and distances, then run:

```bat
python calibrate_camera.py --folder path\to\photos --camera cam1 --square-size 22
```

It finds the checkerboard corners in each photo, runs `cv2.calibrateCamera`, and saves `calibration_cam1.npz` (and a readable `calibration_cam1.json`). For Camera 1 in this repo the result was an RMS reprojection error of about 1.18 pixels over 95 images, which is a good fit for a webcam.

You can see the effect in the before/after images. The left half is the raw frame, the right half is undistorted. Notice how the curved edges become straight.

![Lens undistortion comparison, Camera 1](docs/comparison_cam1.jpg)

### Step 2: Homography (turns straight pixels into table centimeters)

After the lens is corrected, the table is still seen at an angle. The homography maps undistorted pixels to centimeters on the table plane.

`calibrate_homography.py` builds it from four ArUco markers (IDs 0, 1, 2, 3) whose real centre positions you measured with a ruler. You place the markers on the table, the script shows a live preview, and you press SPACE when all four are detected. It then matches each marker corner to its real-world position and computes the 3x3 matrix with `cv2.findHomography`:

```bat
python calibrate_homography.py --camera-index 1
```

The script validates the result against your ruler measurements and prints the mean reprojection error in centimeters. For Camera 1 the mean error is under 1 cm, which is accurate enough to position the hook over a small part. The frame used for this step is saved as `docs/homography_cam1_reference.jpg`.

### A note on the generated file

`homography_cam1.npy` is the **output** of step 2, so it is not committed here. It depends on the exact marker layout used on the day. Run `calibrate_homography.py` once with your setup to create it, then place it next to `detector.py`. The lens file `calibration_cam1.npz` is included because it depends only on the camera and lens, not on the table layout.

---

## Live test tools

These two scripts let me see exactly what the camera sees, which was very useful while building and debugging the vision part. They run on their own, without the LLM or the crane.

- **`live_test.py`** opens the overhead camera and draws the YOLO boxes and the ArUco markers on the live video. This is what I used to check that the model detects the worker, the vehicles, and the markers correctly, and to tune the 0.45 confidence threshold up or down.
- **`live_test_for_camera2.py`** opens the side camera and shows each marker's position in centimeters relative to marker ID 10. It was used to test the table-level view and the pose math.

Run either one and press `q` to close the preview window.

```bat
python live_test.py
```

---

## Running the full system

1. Upload the two sketches with the Arduino IDE: `for_crane_project.ino` to the Uno, `for_nano_crane_magnet.ino` to the Nano.
2. Connect the Uno over USB and note its COM port. Set that port in `controller.py` (the `target_port` value, default `COM10`).
3. Plug in the overhead camera. Check its index. `detector.py` opens index 0, while `server.py` tries index 1 first, then 0. Change these if your camera is on a different index.
4. Make sure `calibration_cam1.npz` and your generated `homography_cam1.npy` are in the project folder.
5. Activate the venv, then start the MCP server:

```bat
venv\Scripts\activate
python server.py
```

6. Connect the MCP server to your LLM client. The model can now call `scan_environment`, `move_crane`, `operate_hook`, and `toggle_magnet`.

---

## Serial command protocol

Python sends short ASCII commands to the Uno. Every command is wrapped between `<` and `>`.

| Command | Meaning |
|---------|---------|
| `<J,body,hook,trolley,duration_ms>` | Drive the three motors at the given speeds for the given time in milliseconds. |
| `<M,1>` / `<M,0>` | Magnet on / magnet off. The Uno relays `'1'` or `'0'` to the Nano. |
| `<S>` | Emergency stop. Halts all motors and drops the magnet. |

The Uno replies with status strings such as `<Ready>`, `<ACK>`, `<DONE>`, `<MAGNET_ON>`, `<MAGNET_OFF>`, and `<STOPPED>`. Motor timing uses `millis()` instead of `delay()`, so the board can still read new commands while a move is running.

---

## Known limits and things to edit first

I would rather be honest about the rough edges than hide them.

- **COM port is hardcoded.** `controller.py` uses `COM10`. Change it to your port before running.
- **Camera index is assumed.** `detector.py` uses index 0 and `server.py` uses index 1 then 0. On a different machine you may need to change these.
- **The fence is not treated as an obstacle.** In `server.py`, `DANGER_OBJECTS` lists `"Fence"` with a capital F, but the check lowercases the detected name first, so a detected fence becomes `"fence"` and never matches. Worker, construction vehicle, and wheelbarrow all work correctly. The one character fix is to change `"Fence"` to `"fence"`. I left it as it is so the published version matches the thesis state.
- **Movement is open loop and time based.** The crane moves a motor for a calculated number of milliseconds based on a measured speed in cm per second. There is no encoder feedback, so position can drift over many moves. The stored position in `crane_memory.json` is an estimate, not a measurement. This is one of the main reasons for the move to stepper motors.
- **The path planner only tries two L-shaped routes.** It is not a full path finder. In a tight cluster of obstacles it can report a safe halt even when a longer route exists.

---

## License and academic use

This code is part of an ongoing Master's thesis. Until the thesis is submitted (deadline 17 August 2026), please treat it as **all rights reserved** and check with me before reusing it. I will add a formal open license after submission, once I have confirmed there is no chair or university rule against publishing thesis code early.

**Author:** Amer El-Saad, M.Sc. Mechatronics, TU Clausthal
**GitHub:** [elsaadamer](https://github.com/elsaadamer)
