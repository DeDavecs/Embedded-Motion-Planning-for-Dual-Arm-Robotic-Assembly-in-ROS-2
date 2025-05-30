#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API for UR5e.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    ur5e = MoveItPy(node_name="moveit_py")
    ur_arm = ur5e.get_planning_component("ur_arm")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan X - set states with predefined string
    ###########################################################################

    # set plan start state using predefined state
    #ur_arm.set_start_state(configuration_name="home")

    # set pose goal using predefined state
    #ur_arm.set_goal_state(configuration_name="extended")

    # Create single pipeline plan request parameters for Pilz PTP planner
    #single_plan_parameters = SinglePipelinePlanRequestParameters(
    #    ur5e,
    #    "pilz_industrial_motion_planner",
    #    "PTP"
    #)

    # plan to goal
    #plan_and_execute(ur5e, ur_arm, logger, sleep_time=3.0)
    
    
    """
    ###########################################################################
    # Plan X - set goal state with RobotState object
    ###########################################################################
    
    # instantiate a RobotState instance using the current robot model
    robot_model = ur5e.get_robot_model()
    robot_state = RobotState(robot_model)

    # randomize the robot state
    robot_state.set_to_random_positions()

    # set plan start state to current state
    ur_arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    ur_arm.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(ur5e, ur_arm, logger, sleep_time=3.0)
    """
    ###########################################################################
    # Plan 1 - set goal state with PoseStamped message
    ###########################################################################
    # instantiate a RobotState instance using the current robot model
    time.sleep(10.0)
    
    robot_model = ur5e.get_robot_model()
    robot_state = RobotState(robot_model)
    # set plan start state to current state
    
    ur_arm.set_start_state_to_current_state()
    
    time.sleep(2.0)
    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "ur5e_base_link"
    pose_goal.pose.orientation.w = 0.7071  # cos(45°)
    pose_goal.pose.orientation.x = 0.0  # sin(45°)
    pose_goal.pose.orientation.y = -0.7071
    pose_goal.pose.orientation.z = 0.0

    pose_goal.pose.position.x = -0.2
    pose_goal.pose.position.y = 0.2
    pose_goal.pose.position.z = 0.8
    ur_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="ur5e_tool0")

    # plan to goal
    plan_and_execute(ur5e, ur_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 2 - set goal state with constraints & joint angles
    ###########################################################################

    # set plan start state to current state
    ur_arm.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_joint_constraint

    joint_values = {
        "ur5e_shoulder_pan_joint": 0.0,
        "ur5e_shoulder_lift_joint": -1.57,
        "ur5e_elbow_joint": 0.0,
        "ur5e_wrist_1_joint": -1.57,
        "ur5e_wrist_2_joint": 0.0,
        "ur5e_wrist_3_joint": 0.0,
    }
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=ur5e.get_robot_model().get_joint_model_group("ur_arm"),
    )
    ur_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_and_execute(ur5e, ur_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan X - Planning with Multiple Pipelines simultaneously
    ###########################################################################
    """
    # set plan start state to current state
    ur_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    ur_arm.set_goal_state(configuration_name="home")

    # initialise multi-pipeline plan request parameters
    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        ur5e, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    )

    # plan to goal
    plan_and_execute(
        ur5e,
        ur_arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=3.0,
    )
    """


if __name__ == "__main__":
    main()