#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__("motion_planner_node")
        self.get_logger().info("Starting MoveItPy...")
        self.ur5e = MoveItPy(node_name="moveit_py")
        self.ur_arm = self.ur5e.get_planning_component("409_ur5e_arm")

        # Subscribe to pose goals
        self.create_subscription(PoseStamped, "pose_goal", self.pose_goal_callback, 10)
        self.get_logger().info("Subscribed to /pose_goal topic")
        
        # Allow MoveIt to initialize
        time.sleep(10.0)

        # Set default start state
        self.robot_model = self.ur5e.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        self.ur_arm.set_goal_state(configuration_name="home")
        self.ur_arm.set_start_state_to_current_state()
        time.sleep(2.0)

    def pose_goal_callback(self, msg):
        self.get_logger().info(f"Received new pose goal: {msg}")
        self.ur_arm.set_start_state_to_current_state()
        self.ur_arm.set_goal_state(pose_stamped_msg=msg, pose_link="409_ur5e_tool0")

        # Plan and execute using both planners as before
        plan_params = MultiPipelinePlanRequestParameters(self.ur5e, ["ompl_rrtc", "pilz_lin"])
        plan_result = self.ur_arm.plan(multi_plan_parameters=plan_params)

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
