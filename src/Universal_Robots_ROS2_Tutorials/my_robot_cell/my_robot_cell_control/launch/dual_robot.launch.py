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
from launch.conditions import IfCondition, UnlessCondition
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
    robot_409_ip = LaunchConfiguration("robot_409_ip")
    robot_410_ip = LaunchConfiguration("robot_410_ip")
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
            "robot_409_ip",
            default_value="192.168.1.100",
            description="IP address of robot 409.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_410_ip",
            default_value="192.168.1.103",
            description="IP address of robot 410.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Robot State Publisher (for the combined URDF)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [
                        FindExecutable(name="xacro"),
                        " ",
                        PathJoinSubstitution([
                            FindPackageShare("my_robot_cell_control"),
                            "urdf",
                            "my_robot_cell_controlled.urdf.xacro"
                        ]),
                        " robot_409_ip:=", robot_409_ip,
                        " robot_410_ip:=", robot_410_ip,
                        " use_mock_hardware:=", use_mock_hardware,
                    ]
                )
            }
        ],
    )

    # ROS2 Control Node (handles both robots from the combined URDF)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "config",
                "ur5e_update_rate.yaml"
            ]),
            PathJoinSubstitution([
                FindPackageShare("my_robot_cell_control"),
                "config",
                "dual_robot_controllers.yaml"
            ]),
        ],
        output="screen",
    )

    # Dashboard clients for both robots
    dashboard_client_409 = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client_409",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[{"robot_ip": robot_409_ip}],
        remappings=[
            ("dashboard_client", "409_dashboard_client"),
        ],
    )

    dashboard_client_410 = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client_410",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[{"robot_ip": robot_410_ip}],
        remappings=[
            ("dashboard_client", "410_dashboard_client"),
        ],
    )

    # Robot state helpers for both robots
    robot_state_helper_409 = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper_409",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {"robot_ip": robot_409_ip},
            {"headless_mode": False},
        ],
    )

    robot_state_helper_410 = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper_410",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {"robot_ip": robot_410_ip},
            {"headless_mode": False},
        ],
    )

    # URScript interfaces for both robots
    urscript_interface_409 = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        name="urscript_interface_409",
        parameters=[{"robot_ip": robot_409_ip}],
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
    )

    urscript_interface_410 = Node(
        package="ur_robot_driver",
        executable="urscript_interface", 
        name="urscript_interface_410",
        parameters=[{"robot_ip": robot_410_ip}],
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
    )

    # Controller stoppers for both robots
    controller_stopper_409 = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper_409",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {"headless_mode": False},
            {"joint_controller_active": True},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster", 
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
    )

    # Spawn controllers
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10",
            "joint_state_broadcaster",
            "io_and_status_controller", 
            "speed_scaling_state_broadcaster",
            "force_torque_sensor_broadcaster",
            "tcp_pose_broadcaster",
            "ur_configuration_controller",
            "scaled_joint_trajectory_controller",
        ],
    )

    # Get MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("my_robot_cell", package_name="moveit_config")
        .to_moveit_configs()
    )

    # Launch MoveGroup
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("moveit_config"),
                "launch",
                "move_group.launch.py",
            ])
        ]),
    )

    # Launch MoveIt RViz
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("moveit_config"),
                "launch", 
                "moveit_rviz.launch.py",
            ])
        ]),
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            control_node,
            dashboard_client_409,
            dashboard_client_410,
            robot_state_helper_409,
            robot_state_helper_410,
            urscript_interface_409,
            urscript_interface_410,
            controller_stopper_409,
            controller_spawner,
            move_group_launch,
            moveit_rviz_launch,
        ]
    ) 