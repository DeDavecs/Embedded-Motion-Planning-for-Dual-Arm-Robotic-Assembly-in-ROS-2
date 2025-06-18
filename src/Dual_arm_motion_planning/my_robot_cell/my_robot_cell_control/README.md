# Package: my_robot_cell_control/README.md

## Overview
Handles the hardware interface and ROS 2 control integration for the dual UR5e system.

## Structure
- `config/`: Controller and calibration configs
- `launch/`: Launch files for starting single or dual robot setups
- `urdf/`: URDF files with control interfaces

## Usage (Simulated Mode)
```bash
ros2 launch my_robot_cell_control dual_robot.launch.py use_mock_hardware:=true
```
- Make sure to adjust ip addresses if necessary.
