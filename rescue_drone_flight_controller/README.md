# Rescue Drone Flight Controller (PX4 + Gazebo SITL)

A **simulation-only** search-and-rescue mission demo for the Fall 2026 Robotics project.
It combines a custom Gazebo world, a PX4 SITL X500 quadcopter, and a Python mission controller.

## What it demonstrates

- Custom Gazebo park: green ground, four low-poly trees, a walking path, and two colored search-zone markers.
- Four mission waypoints defined as north/east offsets in meters from the PX4 simulated home position.
- Automated simulated takeoff, waypoint navigation, and return-to-launch (RTL).
- A **mock** person-detection event at a selected waypoint; **no live camera/person detection is implemented**.

## Setup (macOS / Apple Silicon prototype)

Prerequisites: working local PX4-Autopilot checkout at `~/PX4-Autopilot` with its `.venv`, PX4/macOS Pixi build environment, Gazebo Sim, QGroundControl, and Python 3 with `venv`.
The launcher invokes `xcrun` to locate the installed macOS SDK. Other machines may need local setup changes.

From the `rescue_drone_flight_controller` directory:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-sitl.txt
```

The `start_drone_demo.sh` launcher expects a compatible, working PX4 checkout and Gazebo already installed through the PX4 toolchain; `requirements-sitl.txt` only installs the Python client dependency.

### Start the simulator

```bash
./start_drone_demo.sh
```

Check Gazebo and wait for QGroundControl to show **Ready**. This command keeps a terminal open until the simulator is stopped with Control+C.

### Start the demo mission (second terminal)

```bash
cd /path/to/your/cloned-repo/rescue_drone_flight_controller
source .venv/bin/activate
python flight_controller.py --mode sitl --demo-detection-at 2
```

The demo visits the first two of four planned waypoints, emits a clearly labeled simulated detection at waypoint 2, and requests RTL. **The `COMPLETED` message means PX4 accepted RTL; monitor QGroundControl/Gazebo to verify landing separately.**

For a log-only test that does not launch or control a 3D aircraft:

```bash
python flight_controller.py --mode dry-run --demo-detection-at 2
```

## Route and coordinates

Current route (north m, east m): `(8,0)` → `(8,16)` → `(0,16)` → `(0,0)` at 5 m altitude.
The Gazebo search markers are at world XY `(0,8)` and `(16,8)`; these should be visually near the first two waypoints in the default ENU world, but actual alignment depends on PX4's simulated home origin and should be verified in the simulator.

## Limitations and safety

- Simulation prototype, not flight-ready code for a real drone.
- Detection is mocked and not connected to a real sensor or vision system.
- Trees, path, and markers are mostly visual geometry, not a realistic obstacle/collision environment.
- Simulator tooling may be version-sensitive; full Gazebo/PX4 startup has been tested by the project contributor on a Mac, but **this portable copy has not been end-to-end retested**.
- Do not point the Python controller at a real MAVLink vehicle; it sends commands to any matching connected system.
