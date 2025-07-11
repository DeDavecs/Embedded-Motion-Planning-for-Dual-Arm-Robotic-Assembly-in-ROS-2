# ----------------------------------------------------------------
# Maintainer: Andrin Schälin & David Streib, aschalin@ethz.ch & dstreib@ethz.ch
# Last Updated: 2025-06-15
# Description: Launch file for the dual UR5e robot cell.
# ----------------------------------------------------------------

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

    # Robot State Publisher
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("my_robot_cell_control"),
                "urdf",
                "my_robot_cell_controlled.urdf.xacro"
            ]),
            " ",
            "ur_type:=", ur_type,
            " ",
            "robot_409_ip:=", robot_409_ip,
            " ",
            "robot_410_ip:=", robot_410_ip,
            " ",
            "use_mock_hardware:=", use_mock_hardware,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ROS2 Control Node (handles both robots from the combined URDF)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare("my_robot_cell_control"),
                "config",
                "dual_robot_controllers.yaml"
            ]),
        ],
        output="screen",
    )

    # Dashboard client for robot 409 only (to avoid conflicts)
    dashboard_client_409 = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client_409",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        namespace="robot_409",
        parameters=[{"robot_ip": robot_409_ip}],
    )

    # Dashboard client for robot 410
    dashboard_client_410 = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client_410",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        namespace="robot_410",
        parameters=[{"robot_ip": robot_410_ip}],
    )

    # Robot state helper for robot 409
    robot_state_helper_409 = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper_409",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        namespace="robot_409",
        parameters=[
            {"robot_ip": robot_409_ip},
            {"tf_prefix": "409_ur5e_"},
        ],
    )

    # Robot state helper for robot 410
    robot_state_helper_410 = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper_410",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        namespace="robot_410",
        parameters=[
            {"robot_ip": robot_410_ip},
            {"tf_prefix": "410_ur5e_"},
        ],
    )

    # URScript interface for robot 409
    urscript_interface_409 = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        name="urscript_interface_409",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        namespace="robot_409",
        parameters=[{"robot_ip": robot_409_ip}],
    )

    # URScript interface for robot 410
    urscript_interface_410 = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        name="urscript_interface_410",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        namespace="robot_410",
        parameters=[{"robot_ip": robot_410_ip}],
    )

    # Controller stopper for robot 409
    controller_stopper_409 = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper_409",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        namespace="robot_409",
        parameters=[
            {"headless_mode": False},
            {"joint_controller_active": True},
            {"consistent_controllers": [
                "io_and_status_controller",
                "force_torque_sensor_broadcaster", 
                "joint_state_broadcaster",
                "speed_scaling_state_broadcaster",
                "scaled_joint_trajectory_controller",
            ]},
        ],
    )

    # Controller stopper for robot 410
    controller_stopper_410 = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper_410",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        namespace="robot_410",
        parameters=[
            {"headless_mode": False},
            {"joint_controller_active": True},
            {"consistent_controllers": [
                "robot_410_scaled_joint_trajectory_controller",
            ]},
        ],
    )

    # Spawn controllers for robot 409
    controller_spawner_409 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10",
            "joint_state_broadcaster",
            "io_and_status_controller", 
            "speed_scaling_state_broadcaster",
            "force_torque_sensor_broadcaster",
            "scaled_joint_trajectory_controller",
        ],
    )

    # Spawn controllers for robot 410 (simplified - only the trajectory controller for now)
    controller_spawner_410 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "15",
            "robot_410_scaled_joint_trajectory_controller",
        ],
    )

    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder("my_robot_cell", package_name="moveit_config")
        .robot_description(file_path="config/my_robot_cell.urdf.xacro")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("motion_planning"),
                "config",
                "motion_planning_python_api.yaml"
            )
        )
        .to_moveit_configs()
    )

    # MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # MoveIt RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("moveit_config"),
        "config",
        "moveit.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Create the motion planning Python API node
    moveit_py_node = Node(
        name="moveit_py",
        package="motion_planning",
        executable="motion_planning_python_api.py",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    # Create the launch description
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
            controller_stopper_410,
            controller_spawner_409,
            controller_spawner_410,
            move_group_node,
            rviz_node,
            moveit_py_node,
        ]
    )