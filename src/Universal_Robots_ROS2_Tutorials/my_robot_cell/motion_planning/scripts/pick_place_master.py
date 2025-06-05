import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class PickPlaceMaster(Node):
    def __init__(self):
        super().__init__('pick_place_master')
        self.publisher = self.create_publisher(PoseStamped, 'pose_goal', 10)
        self.timer = self.create_timer(1.0, self.run_sequence)
        self.sequence_step = 0
        self.executing = False

        # Define your sequence of poses
        self.poses = [
            self.make_pose(-0.5, 0.4, 0.4),  # Pre-grasp
            self.make_pose(-0.5, 0.4, 0.3),  # Grasp
            # You could call a gripper service here
            self.make_pose(-0.4, 0.5, 0.4),   # Pre-place
            self.make_pose(-0.4, 0.5, 0.3),   # Place
            # Open gripper, then home
            self.make_pose(0.0, 0.0, 1.0),   # Home
        ]

    def make_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "ur5e_base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Facing down:
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose

    def run_sequence(self):
        if self.executing:
            return
        if self.sequence_step >= len(self.poses):
            self.get_logger().info("Sequence complete!")
            self.destroy_node()
            return
        # Publish the next pose
        self.get_logger().info(f"Publishing pose {self.sequence_step + 1}")
        self.publisher.publish(self.poses[self.sequence_step])
        self.sequence_step += 1
        self.executing = True
        # Wait for execution (simulate)
        time.sleep(3.0)  # Adjust based on robot speed/plan duration
        self.executing = False

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceMaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
