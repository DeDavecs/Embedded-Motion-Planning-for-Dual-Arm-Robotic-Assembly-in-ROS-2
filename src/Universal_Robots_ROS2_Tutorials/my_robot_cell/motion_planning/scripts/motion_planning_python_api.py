#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__("motion_planner_node")
        self.get_logger().info("Starting MoveItPy...")
        self.ur5e = MoveItPy(node_name="moveit_py")
        
        # Get both robot arms
        self.ur_arm_409 = self.ur5e.get_planning_component("409_ur5e_arm")
        self.ur_arm_410 = self.ur5e.get_planning_component("410_ur5e_arm")

        # Subscribe to pose goals for both robots
        self.create_subscription(PoseStamped, "pose_goal_409", self.pose_goal_409_callback, 10)
        self.create_subscription(PoseStamped, "pose_goal_410", self.pose_goal_410_callback, 10)
        self.create_subscription(String, "robot_select", self.robot_select_callback, 10)
        
        self.get_logger().info("Subscribed to /pose_goal_409, /pose_goal_410, and /robot_select topics")
        
        # Current active robot (default to 409)
        self.active_robot = "409"
        
        # Allow MoveIt to initialize
        time.sleep(10.0)

        # Set default start states
        self.robot_model = self.ur5e.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        
        # Set both robots to home position
        self.ur_arm_409.set_goal_state(configuration_name="home")
        self.ur_arm_409.set_start_state_to_current_state()
        
        self.ur_arm_410.set_goal_state(configuration_name="home")
        self.ur_arm_410.set_start_state_to_current_state()
        
        time.sleep(2.0)

    def robot_select_callback(self, msg):
        if msg.data in ["409", "410"]:
            self.active_robot = msg.data
            self.get_logger().info(f"Switched to robot {self.active_robot}")

    def pose_goal_409_callback(self, msg):
        self.get_logger().info(f"Received pose goal for robot 409: {msg}")
        self.execute_motion_plan(self.ur_arm_409, msg, "409_ur5e_tool0")

    def pose_goal_410_callback(self, msg):
        self.get_logger().info(f"Received pose goal for robot 410: {msg}")
        self.execute_motion_plan(self.ur_arm_410, msg, "410_ur5e_tool0")

    def execute_motion_plan(self, arm, pose_msg, tool_frame):
        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=pose_msg, pose_link=tool_frame)

        # Plan and execute using both planners
        plan_params = MultiPipelinePlanRequestParameters(self.ur5e, ["ompl_rrtc", "pilz_lin"])
        plan_result = arm.plan(multi_plan_parameters=plan_params)

        if plan_result:
            self.get_logger().info("Executing plan")
            self.ur5e.execute(plan_result.trajectory, controllers=[])
        else:
            self.get_logger().error("Planning failed")

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()