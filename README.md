# Embedded Motion Planning for Dual-Arm Robotic Assembly in ROS 2
Bimanual robot motion planning using ros2-jazzy in moveit. The goal is the assembly of wooden objects from individual pieces using 2 Universal Robots UR5e and a onrobot 2fg7 gripper. Also, the setup is structured in a modular way, that makes it possible to add different end effectors. 

## Features
- Dual UR5e setup with gripper integration
- Modular architecture with 4 ROS 2 packages
- Python-based motion planning API
- Simulation and real-hardware support


## What you need to get started
* ### ROS 2 - jazzy installation
  Install ROS 2 Jazzy [here](https://docs.ros.org/en/jazzy/Installation.html)
  
  #### Test your installation by running
  ```
   ros2
   ```
  
  To facilitate sourcing your ros2 installation each time, put the following in your .bashrc

  ```
  source /opt/ros/jazzy/setup.bash
  ```
  
* ### MoveIT installation (from source)

  Install MoveIt2 [here](https://moveit.ai/install-moveit2/source/)
  
  Again, make sure to source the workspace or put in you .bashrc
  
  ```
  source $COLCON_WS/install/setup.bash
  ```
## Installing this repository:
* Clone the repository int the src of your colcon workspace:
  ```
  git clone: https://github.com/DeDavecs/Embedded-Motion-Planning-for-Dual-Arm-Robotic-Assembly-in-ROS-2.git
  ```
* Build the workspace. Within you workspace bash:
  ```
  colcon build
  ```
  You might have to run colcon again, until all packages are succesfully installed.
  
  
* Source the workspace. Bash:
  ```
  source install/setup.bash
  ```
  
* Start the graphical interface RViz and launch MoveIt with:
  ```
  ros2 launch my_robot_cell_control dual_robot.launch.py use_mock_hardware:=true
  ```
  
  Note, you can add/change the following arguments, depending on the ip adress or if your connected to real hardware:
    ```
     robot_ip_XY:=<my ip address XY>
    ```
  ```
  use_mock_hardware:= false
  ```

* Now, you can launch the demo script or your own Python_API built motion plan. For instance, launch:
  ```
  ros2 run motion_planning pick_and_place_example.py 
  ```
