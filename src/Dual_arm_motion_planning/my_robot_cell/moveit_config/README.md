# Package: moveit_config/README.md

## Overview
Contains the MoveIt 2 planning configuration for the dual-arm setup.

## Structure
- `config/`: SRDF, joint limits, controllers, kinematics
- `launch/`: Launch files for MoveIt components

## Usage
```bash
ros2 launch moveit_config move_group.launch.py
```
- Use these launch files only for testing. You can find the general launch file in my_robot_cell_control
