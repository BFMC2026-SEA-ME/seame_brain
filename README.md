# SEA:ME BFMC Dashboard

This repository contains the SEA:ME team's dashboard for the BFMC autonomous vehicle. It is customized from the official BFMC Brain project to fit the SEA:ME vehicle architecture, ROS2 pipeline, and Jetson runtime environment.

- Original BFMC Brain repository: https://github.com/ECC-BFMC/Brain
- The original project provides Raspberry Pi-based vehicle control, Nucleo communication, sensor data handling, environmental server APIs, and simulated servers.
- This repository keeps the original BFMC Brain structure while adding SEA:ME-specific dashboard and ROS2 integration features.

## Key Changes

- `Brain/main.py` starts the dashboard, camera, semaphore/traffic communication, serial handler, Ackermann bridge, and global planning bridge together.
- Added a ROS2 RealSense compressed image subscriber that forwards camera frames to the dashboard live camera view.
- Added an Ackermann bridge that converts `/ackermann_cmd` messages into speed/steer commands used by the BFMC serial handler.
- Added a global planning bridge for goal node IDs, global paths, global poses, and ordered checkpoints between ROS2 and the dashboard.
- Extended the dashboard API and WebSocket flow to display GraphML map nodes, traffic light/semaphore states, and vehicle pose on the map UI.
- Added vehicle computer monitoring data such as Jetson CPU temperature, CPU usage, memory usage, and network status.

## Setup

Run commands from the `Brain` directory.

```bash
cd Brain
```

To use the setup script:

```bash
chmod +x setup.sh
./setup.sh
```

For manual installation:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt

cd src/dashboard/frontend
npm install
cd ../../..
```

To run with the ROS2 bridge features enabled, the ROS2 environment and message packages must be available first. The default `main.py` configuration enables the Ackermann bridge, so the runtime environment must provide packages such as `rclpy`, `ackermann_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, and `sensor_msgs`.

Example:

```bash
source /opt/ros/humble/setup.bash
source <your_ros2_ws>/install/setup.bash
```

## Running

Start the backend processes from `main.py`.

```bash
cd Brain
source .venv/bin/activate
python3 main.py
```

Start the dashboard frontend in a separate terminal.

```bash
cd Brain/src/dashboard/frontend
npm start
```

Open the dashboard in a browser.

```text
http://localhost:4200
```

The dashboard backend is started by `processDashboard` from `main.py` and listens on `0.0.0.0:5005`. The frontend communicates with this backend through WebSocket and HTTP API calls.

## Runtime Options

The process enable flags are defined at the top of `Brain/main.py`.

```python
ENABLE_GATEWAY = True
ENABLE_DASHBOARD = True
ENABLE_CAMERA = True
ENABLE_SEMAPHORES = True
ENABLE_TRAFFIC_COM = True
ENABLE_SERIAL_HANDLER = True
ENABLE_CMDVELBRIDGE = False
ENABLE_ACKERMAN = True
ENABLE_GLOBAL_PLANNING_BRIDGE = True
```

Common environment variables:

```bash
export TRAFFIC_GPS_CAR_ID=5
export ROS_CAMERA_TOPIC=/d455f/d455f/color/image_raw/compressed
export ROS_CAMERA_MAX_FPS=1
export GLOBAL_PLANNING_GRAPHML=/path/to/track2.graphml
```

To check only the UI without ROS2 or vehicle hardware, set the unnecessary bridge, camera, and serial flags in `main.py` to `False` before running.

## Shutdown

Stop the backend and frontend terminals with `Ctrl + C`. When `main.py` receives `KeyboardInterrupt`, it shuts down the running child processes in order.
