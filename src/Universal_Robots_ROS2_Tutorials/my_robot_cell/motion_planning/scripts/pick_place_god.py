#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ur_msgs.srv import SetIO
import time

class PoseGoalPublisher(Node):
    def __init__(self):
        super().__init__("pose_goal_publisher")
        self.publisher_409 = self.create_publisher(PoseStamped, "pose_goal_409", 10)
        self.publisher_410 = self.create_publisher(PoseStamped, "pose_goal_410", 10)
        
        # Create service client for gripper control
        self.gripper_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        
        # Subscribe to the motion result topic
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'motion_status', self.motion_result_callback, qos_profile)
        
        # --- State Variables ---
        self.is_waiting_for_result = False
        self.counter = 0
        self.total_poses = 9  # Fixed: Steps 0-8 = 9 total steps

        # The timer drives the sequence execution
        timer_period = 2.0  # Reduced timer period for more responsive checking
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("🚀 Pose Goal Publisher initialized. Starting sequence execution...")

    def motion_result_callback(self, msg):
        """This function is called when the planner reports a result."""
        if not self.is_waiting_for_result:
            self.get_logger().debug("Ignoring unexpected motion status message")
            return # Ignore unexpected messages

        if msg.data == 'SUCCEEDED':
            self.get_logger().info(f"✅ Step {self.counter} COMPLETED SUCCESSFULLY! Moving to next pose...")
            self.counter += 1
            self.is_waiting_for_result = False
        elif msg.data == 'FAILED':
            self.get_logger().error(f"❌ Step {self.counter} FAILED after all retry attempts. This should not happen with continuous retry!")
            self.is_waiting_for_result = False
        else:
            self.get_logger().warn(f"Unknown motion status received: {msg.data}")

    def close_gripper(self):
        """Call the service to close the gripper."""
        if not self.gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Gripper service not available!")
            # Treat as success to continue sequence, but log the error
            self.counter += 1
            self.is_waiting_for_result = False
            return

        request = SetIO.Request()
        request.fun = 1      # Digital output
        request.pin = 0      # Pin 0
        request.state = 1.0  # Close gripper (set to 1)

        future = self.gripper_client.call_async(request)
        future.add_done_callback(lambda f: self.gripper_response_callback(f, "close"))

    def open_gripper(self):
        """Call the service to open the gripper."""
        if not self.gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Gripper service not available!")
            # Treat as success to continue sequence, but log the error
            self.counter += 1
            self.is_waiting_for_result = False
            return

        request = SetIO.Request()
        request.fun = 1      # Digital output
        request.pin = 0      # Pin 0
        request.state = 0.0  # Open gripper (set to 0)

        future = self.gripper_client.call_async(request)
        future.add_done_callback(lambda f: self.gripper_response_callback(f, "open"))

    def gripper_response_callback(self, future, operation):
        """Handle the gripper service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ Gripper {operation}ed successfully!")
            else:
                self.get_logger().error(f"❌ Failed to {operation} gripper!")
            
            # Move to next step regardless of gripper result
            self.counter += 1
            self.is_waiting_for_result = False
            
        except Exception as e:
            self.get_logger().error(f"❌ Gripper service call failed: {e}")
            # Move to next step even if service failed
            self.counter += 1
            self.is_waiting_for_result = False

    def timer_callback(self):
        """This function attempts to execute the next step."""
        if self.is_waiting_for_result:
            self.get_logger().debug("Waiting for current motion to complete...")
            return

        if self.counter >= self.total_poses:
            self.get_logger().info("🎉🎉🎉 ALL POSES COMPLETED SUCCESSFULLY! 🎉🎉🎉")
            self.timer.cancel()
            return

        msg = PoseStamped()
        msg.header.frame_id = "table"
        msg.header.stamp = self.get_clock().now().to_msg()  # Add timestamp

        if self.counter == 0:
            self.get_logger().info(f"📍 Executing pose {self.counter + 1}/{self.total_poses}: Robot 409 to position (0.1, 0.15, 0.27)")
            msg.pose.position.x = 0.10
            msg.pose.position.y = 0.15
            msg.pose.position.z = 0.27
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            self.publisher_409.publish(msg)
            self.is_waiting_for_result = True
            
        elif self.counter == 1:
            self.get_logger().info(f"📍 Executing pose {self.counter + 1}/{self.total_poses}: Robot 410 to position (-0.10, -0.10, 0.27)")
            msg.pose.position.x = -0.20
            msg.pose.position.y = -0.20
            msg.pose.position.z = 0.27
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            self.publisher_410.publish(msg)
            self.is_waiting_for_result = True

        elif self.counter == 2:
            self.get_logger().info(f"📍 Executing pose {self.counter + 1}/{self.total_poses}: Robot 409 to position (-0.10, -0.10, 0.22)")
            msg.pose.position.x = 0.10
            msg.pose.position.y = 0.15
            msg.pose.position.z = 0.22
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            self.publisher_409.publish(msg)
            self.is_waiting_for_result = True

        elif self.counter == 3:
            self.get_logger().info(f"🤖 Executing step {self.counter + 1}/{self.total_poses}: Closing gripper")
            self.is_waiting_for_result = True
            self.close_gripper()

        elif self.counter == 4:
            self.get_logger().info(f"📍 Executing pose {self.counter + 1}/{self.total_poses}: Robot 409 to position (0.10, 0.15, 0.27)")
            msg.pose.position.x = 0.10
            msg.pose.position.y = 0.15
            msg.pose.position.z = 0.27
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            self.publisher_409.publish(msg)
            self.is_waiting_for_result = True


        elif self.counter == 5:
            self.get_logger().info(f"📍 Executing pose {self.counter + 1}/{self.total_poses}: Robot 410 to position (-0.10, -0.10, 0.27)")
            msg.pose.position.x = 0.20
            msg.pose.position.y = -0.20
            msg.pose.position.z = 0.6
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            self.publisher_410.publish(msg)
            self.is_waiting_for_result = True

        elif self.counter == 6:
            self.get_logger().info(f"📍 Executing pose {self.counter + 1}/{self.total_poses}: Robot 409 to position (-0.3, -0.3, 0.27)")
            msg.pose.position.x = 0.1
            msg.pose.position.y = -0.2
            msg.pose.position.z = 0.4
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            self.publisher_409.publish(msg)
            self.is_waiting_for_result = True



        elif self.counter == 7:
            self.get_logger().info(f"📍 Executing pose {self.counter + 1}/{self.total_poses}: Robot 409 to position (-0.3, -0.3, 0.27)")
            msg.pose.position.x = 0.1
            msg.pose.position.y = -0.2
            msg.pose.position.z = 0.22
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            self.publisher_409.publish(msg)
            self.is_waiting_for_result = True



        elif self.counter == 8:
            self.get_logger().info(f"🤖 Executing step {self.counter + 1}/{self.total_poses}: Opening gripper (letting go)")
            self.is_waiting_for_result = True
            self.open_gripper()

def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down pose goal publisher...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()