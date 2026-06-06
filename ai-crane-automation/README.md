# 🏗️ AI-Controlled Tower Crane

### Physical AI: a real crane you operate by talking to it.

A language model looks at the workspace through a camera, decides what to do, and drives a physical tower crane to do it. It lifts real objects with an electromagnet, plans its own path, and will not move through a person even when told to.

![Physical AI](https://img.shields.io/badge/Physical_AI-vision_%2B_LLM_%2B_hardware-6f42c1)
![Python](https://img.shields.io/badge/Python-3.11-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-vision-5C3EE8?logo=opencv&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLOv8-detection-00b8d4)
![Arduino](https://img.shields.io/badge/Arduino-C++-00979D?logo=arduino&logoColor=white)
![MCP](https://img.shields.io/badge/MCP-LLM_control-FF6F00)

![What the camera sees: YOLOv8 detections and ArUco markers](docs/yolo_detection.png)

## What it does

- **Takes orders in plain language.** "Pick up the screw and move it to Station B" becomes real motion.
- **Sees in real units.** A single overhead camera plus YOLOv8, ArUco markers, and a homography turn the live image into true centimeters on the table.
- **Plans and acts.** It chooses a safe route, drives three motors, and switches an electromagnet to carry the load.
- **Will not hurt anyone.** A hard safety layer blocks any path that passes too close to a worker. The model cannot override it.

## See it in action

> **You:** move to Station B
> **Crane:** The direct path is blocked by the worker at (8.6, -2.1). Routing around them.
> **Crane:** Clear path going south. Now swinging east, then up to Station B.
> **Crane:** Arrived at Station B (19.0, -1.4).

The worker never gets run over, because that rule lives in the control code, not in the model's good intentions.

![The physical system: crane, overhead camera, and workspace](docs/crane_setup.jpg)

## Why it is interesting

- **The AI reasons, the code keeps it safe.** The language model can plan and talk, but it physically cannot drive the crane through a person, because the safety check sits in the control layer below it.
- **Fault tolerant by design.** Control is split across two microcontrollers, so the magnet keeps holding its load even if the main board resets.
- **Built end to end by one person.** Mechanics, electronics, C++ firmware, the Python vision and control stack, and the LLM integration.

## How it works

- **Vision:** YOLOv8 (six classes) finds the objects, ArUco markers (DICT_4X4_50) fix the stations and origin, and a homography maps pixels to centimeters. Python and OpenCV.
- **Control:** Arduino Uno and Nano in C++, three motors, and an electromagnet, over a small serial protocol.
- **Brain:** a Large Language Model connected through the [Model Context Protocol (MCP)](https://modelcontextprotocol.io), calling four safe tools: scan, move, hook, magnet.

## Tech stack

`Python` · `OpenCV` · `YOLOv8 (Ultralytics)` · `ArUco` · `NumPy` · `Arduino C++` · `PySerial` · `Model Context Protocol`

## Run it

The full code, the trained model, the camera calibration, and a complete step-by-step guide are in this repository. See **[SETUP.md](SETUP.md)** for installation and how to drive the crane from natural language.

## About

This is the practical part of my Master's thesis at TU Clausthal (Chair of Intelligent Automation Systems). This repository is the DC motor prototype. The crane is being rebuilt with a sturdier structure and NEMA 17 stepper motors with TB6600 drivers for more accurate positioning, and that version will follow as a separate release.

**Amer El-Saad**, M.Sc. Mechatronics, TU Clausthal
[LinkedIn](https://linkedin.com/in/amerelsaad) · [GitHub](https://github.com/elsaadamer)
