from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    # Declare arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Get MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("my_robot_cell", package_name="moveit_config")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("motion_planning"),
                "config",
                "motion_planning_python_api.yaml"
            )
        )
        .to_moveit_configs()
    )

    # Create the motion planning Python API node
    moveit_py_node = Node(
        name="moveit_py",
        executable="python3",
        arguments=[os.path.join(get_package_share_directory("motion_planning"), "scripts", "motion_planning_python_api.py")],
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    # Create the launch description
    return LaunchDescription(
        declared_arguments
        + [
            # Start the robot with mock hardware
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ur_robot_driver"),
                                "launch",
                                "ur_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "tf_prefix": [LaunchConfiguration("ur_type"), "_"],
                    "rviz_config_file": PathJoinSubstitution(
                        [
                            FindPackageShare("my_robot_cell_description"),
                            "rviz",
                            "urdf.rviz",
                        ]
                    ),
                    "description_launchfile": PathJoinSubstitution(
                        [
                            FindPackageShare("my_robot_cell_control"),
                            "launch",
                            "rsp.launch.py",
                        ]
                    ),
                    "use_mock_hardware": use_mock_hardware,
                }.items(),
            ),
            # Launch MoveGroup
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("moveit_config"),
                                "launch",
                                "move_group.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            # Launch MoveIt RViz
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("moveit_config"),
                                "launch",
                                "moveit_rviz.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            # Add the motion planning Python API node
            moveit_py_node,
        ]
    ) 
