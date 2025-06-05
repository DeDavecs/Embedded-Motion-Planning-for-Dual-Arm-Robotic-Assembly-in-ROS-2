import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseGoalPublisher(Node):
    def __init__(self):
        super().__init__("pose_goal_publisher")
        self.publisher = self.create_publisher(PoseStamped, "pose_goal", 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.has_published = False

    def timer_callback(self):
        if self.has_published:
            return
        msg = PoseStamped()
        msg.header.frame_id = "ur5e_base_link"
        msg.pose.position.x = -0.2
        msg.pose.position.y = 0.3
        msg.pose.position.z = 0.8
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher.publish(msg)
        self.get_logger().info("Published pose goal!")
        self.has_published = True

def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
