# Package: my_robot_cell_description/README.md

## Overview
Defines the kinematic and visual model of the dual-arm robot cell (URDF/Xacro).

## Structure
- `urdf/`: Xacro and URDF files for robots and environment
- `launch/`: Launch files to visualize robot model in RViz
- `rviz/`: RViz config

## Usage
```bash
ros2 launch my_robot_cell_description view_robot.launch.py
```
- Use this launch file only for testing. You can find the complete launch file at my_robt_cell_control
