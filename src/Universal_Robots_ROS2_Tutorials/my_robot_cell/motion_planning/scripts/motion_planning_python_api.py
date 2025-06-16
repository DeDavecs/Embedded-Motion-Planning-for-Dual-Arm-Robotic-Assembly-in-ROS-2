#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
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

        # Publisher for motion status feedback
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(String, "motion_status", qos_profile)

        # Subscribe to pose goals for both robots
        self.create_subscription(PoseStamped, "pose_goal_409", self.pose_goal_409_callback, 10)
        self.create_subscription(PoseStamped, "pose_goal_410", self.pose_goal_410_callback, 10)
        self.create_subscription(String, "robot_select", self.robot_select_callback, 10)
        
        self.get_logger().info("Subscribed to /pose_goal_409, /pose_goal_410, and /robot_select topics")
        
        # Current active robot (default to 409)
        self.active_robot = "409"
        
        # Planning parameters
        self.max_planning_attempts = 50  # Maximum attempts before giving up
        self.planning_retry_delay = 1.0  # Delay between retries in seconds
        
        # Allow MoveIt to initialize
        time.sleep(10.0)

        # Set default start states
        self.robot_model = self.ur5e.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        
        time.sleep(2.0)

    def robot_select_callback(self, msg):
        if msg.data in ["409", "410"]:
            self.active_robot = msg.data
            self.get_logger().info(f"Switched to robot {self.active_robot}")

    def pose_goal_409_callback(self, msg):
        self.get_logger().info(f"Received pose goal for robot 409: {msg}")
        self.execute_motion_plan_with_retry(self.ur_arm_409, msg, "409_ur5e_tool0", "409")

    def pose_goal_410_callback(self, msg):
        self.get_logger().info(f"Received pose goal for robot 410: {msg}")
        self.execute_motion_plan_with_retry(self.ur_arm_410, msg, "410_ur5e_tool0", "410")

    def execute_motion_plan_with_retry(self, arm, pose_msg, tool_frame, robot_id):
        """Execute motion plan with continuous retries until success."""
        attempt = 1
        
        while attempt <= self.max_planning_attempts:
            self.get_logger().info(f"Planning attempt {attempt}/{self.max_planning_attempts} for robot {robot_id}")
            
            success = self.execute_motion_plan(arm, pose_msg, tool_frame)
            
            if success:
                self.get_logger().info(f"✅ Motion planning SUCCEEDED for robot {robot_id} on attempt {attempt}")
                # Publish success status
                status_msg = String()
                status_msg.data = "SUCCEEDED"
                self.status_publisher.publish(status_msg)
                return
            else:
                self.get_logger().warn(f"❌ Motion planning FAILED for robot {robot_id} on attempt {attempt}")
                if attempt < self.max_planning_attempts:
                    self.get_logger().info(f"Retrying in {self.planning_retry_delay} seconds...")
                    time.sleep(self.planning_retry_delay)
                attempt += 1
        
        # If we reach here, all attempts failed
        self.get_logger().error(f"🚫 CRITICAL: Motion planning FAILED for robot {robot_id} after {self.max_planning_attempts} attempts")
        status_msg = String()
        status_msg.data = "FAILED"
        self.status_publisher.publish(status_msg)

    def execute_motion_plan(self, arm, pose_msg, tool_frame):
        """Execute a single motion plan attempt. Returns True if successful, False otherwise."""
        try:
            arm.set_start_state_to_current_state()
            arm.set_goal_state(pose_stamped_msg=pose_msg, pose_link=tool_frame)

            # Plan using both planners - these already include collision checking
            plan_params = MultiPipelinePlanRequestParameters(self.ur5e, ["ompl_rrtc", "pilz_lin"])
            plan_result = arm.plan(multi_plan_parameters=plan_params)

            if plan_result:
                robot_trajectory = plan_result.trajectory
                
                self.get_logger().info("Plan found, executing...")
                
                # Execute the trajectory
                success = self.ur5e.execute(robot_trajectory, controllers=[])
                
                if success:
                    self.get_logger().info("Execution completed successfully")
                    return True
                else:
                    self.get_logger().error("Execution failed")
                    return False
            else:
                self.get_logger().warn("No valid plan found - may be due to collision detection or unreachable goal")
                return False
        
        except Exception as e:
            self.get_logger().error(f"Exception during planning/execution: {str(e)}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()