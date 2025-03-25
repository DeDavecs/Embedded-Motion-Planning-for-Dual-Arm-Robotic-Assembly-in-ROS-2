# UR5e_Assembly_of_Wooden_Pieces
Bimanual robot motion planning using ros2-jazzy in gazebo and moveit. Our goal is the assembly of wooden objects  from individual pieces using screws. 

## The following link provides useful commands
https://docs.google.com/document/d/1OQLkVzkLvqzL7v4BQNEu1JERFHc08q8_XpsmeVT9Lkc/edit?usp=sharing

## What you need to get started
* ### ros2 - jazzy installation
  https://docs.ros.org/en/jazzy/Installation.html
  #### Test your installation by running
  ```
   ros2
   ```
* ### Gazebo physics simulator
  https://gazebosim.org/docs/harmonic/ros_installation/
  #### Test your installation by running
  ```
   gz sim
  ```
* ### MoveIT installation (and workspace)
  https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html#install-ros-2-and-colcon
  #### Test your installation by running
  ```
   ros2 launch moveit2_tutorials demo.launch.py
  ```

## Useful commands
```
   ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
```

