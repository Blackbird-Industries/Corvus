# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Corvus is an autonomous drone ground control station (GCS) built for Raspberry Pi 5 by Blackbird Industries. It integrates an IMX415 camera with AprilTag detection, Pixhawk flight controller telemetry via MAVSDK, and a Crow-based web dashboard for real-time monitoring.

## Build Commands

```bash
# From RaspberryPi/ directory:
cd RaspberryPi && mkdir -p build && cd build
cmake ..
make -j$(nproc)

# Run the binary (on Pi 5 with camera + Pixhawk connected):
./corvus_gcs
```

The web dashboard serves on `0.0.0.0:5000`.

## Dependencies

All installed via system packages on Raspberry Pi OS:
- **OpenCV** — camera capture, frame encoding, drawing
- **MAVSDK** — MAVLink telemetry from Pixhawk over `/dev/serial0` at 57600 baud
- **AprilTag** — tag36h11 fiducial detection (via pkg-config)
- **GStreamer 1.0** — hardware camera pipeline for Pi 5 IMX415
- **Crow** — bundled single-header HTTP/WebSocket server (`crow_all.h`)

## Architecture

The system runs three concurrent components in `main.cpp`:

1. **camera_worker thread** — GStreamer pipeline captures NV12 frames at 640×480, converts to BGR for OpenCV, runs AprilTag detection, encodes to JPEG, and stores in a mutex-protected single-frame buffer. Detected tags trigger actions via `TagAction::execute()` with a 3-second per-tag debounce.

2. **telemetry_worker thread** — Connects to the Pixhawk via MAVSDK, subscribes to position (2 Hz) and attitude (5 Hz), and writes altitude/roll/pitch/battery to the shared `drone_state` struct under mutex.

3. **Crow web server** — Serves the HTML dashboard (`GET /`), current JPEG frame (`GET /snapshot`), and real-time telemetry over WebSocket (`/ws` at 200ms polling).

All shared state flows through `drone_state` (mutex-protected struct) and `global_frame_buffer` (mutex-protected JPEG bytes).

## Key Files

- `RaspberryPi/main.cpp` — Core application: camera, telemetry, web server, AprilTag processing
- `RaspberryPi/TagAction.hpp` — Maps AprilTag IDs to descriptions and actions (ID 0=HOME BASE, 10=SUPPLY DROP, 20=WAYPOINT)
- `RaspberryPi/index.html` — Indigo-themed dashboard with video feed, telemetry cards, WebSocket client
- `RaspberryPi/crow_all.h` — Bundled Crow HTTP framework (do not edit)
- `RaspberryPi/CMakeLists.txt` — Build config targeting C++17 with `-O3 -march=native`

## Branch Structure

- `main` — Production branch (merged Motor-Control)
- `origin/Object-Detection` — Latest development: AprilTag detection, Indigo UI, real Pixhawk telemetry, TagAction system. Significantly ahead of main.

## Hardware Target

Raspberry Pi 5 with IMX415 CSI camera and Pixhawk flight controller over UART serial.
