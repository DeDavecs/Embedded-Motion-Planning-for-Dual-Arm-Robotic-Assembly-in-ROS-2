import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class PoseGoalPublisher(Node):
    def __init__(self):
        super().__init__("pose_goal_publisher")
        self.publisher_409 = self.create_publisher(PoseStamped, "pose_goal_409", 10)
        self.publisher_410 = self.create_publisher(PoseStamped, "pose_goal_410", 10)
        self.robot_selector = self.create_publisher(String, "robot_select", 10)
        
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        if self.counter == 0:
            # First, send a goal to robot 409
            msg = PoseStamped()
            msg.header.frame_id = "table"
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = 0.5
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            self.publisher_409.publish(msg)
            self.get_logger().info("Published pose goal for robot 409!")
            
        elif self.counter == 1:
            # Then, send a goal to robot 410
            msg = PoseStamped()
            msg.header.frame_id = "table"
            msg.pose.position.x = 0.2
            msg.pose.position.y = -0.3
            msg.pose.position.z = 0.8
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            self.publisher_410.publish(msg)
            self.get_logger().info("Published pose goal for robot 410!")
            
        self.counter += 1
        if self.counter > 1:
            self.counter = 0  # Reset to cycle between robots

def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()