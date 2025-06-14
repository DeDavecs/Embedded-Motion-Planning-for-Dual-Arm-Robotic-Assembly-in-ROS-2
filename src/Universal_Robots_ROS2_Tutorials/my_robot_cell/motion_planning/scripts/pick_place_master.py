import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class PickPlaceMaster(Node):
    def __init__(self):
        super().__init__('pick_place_master')
        
        # Publishers for both robots
        self.publisher_409 = self.create_publisher(PoseStamped, 'pose_goal_409', 10)
        self.publisher_410 = self.create_publisher(PoseStamped, 'pose_goal_410', 10)
        self.robot_selector = self.create_publisher(String, 'robot_select', 10)
        
        self.timer = self.create_timer(1.0, self.run_sequence)
        self.sequence_step = 0
        self.executing = False
        self.current_robot = "409"  # Start with robot 409

        # Define your sequence of poses for robot 409
        self.poses_409 = [
            self.make_pose(-0.5, 0.4, 0.4, "409_ur5e_base_link"),  # Pre-grasp
            self.make_pose(-0.5, 0.4, 0.3, "409_ur5e_base_link"),  # Grasp
            self.make_pose(-0.4, 0.5, 0.4, "409_ur5e_base_link"),  # Pre-place
            self.make_pose(-0.4, 0.5, 0.3, "409_ur5e_base_link"),  # Place
            self.make_pose(0.0, 0.0, 1.0, "409_ur5e_base_link"),   # Home
        ]
        
        # Define your sequence of poses for robot 410
        self.poses_410 = [
            self.make_pose(0.5, -0.4, 0.4, "410_ur5e_base_link"),  # Pre-grasp
            self.make_pose(0.5, -0.4, 0.3, "410_ur5e_base_link"),  # Grasp
            self.make_pose(0.4, -0.5, 0.4, "410_ur5e_base_link"),  # Pre-place
            self.make_pose(0.4, -0.5, 0.3, "410_ur5e_base_link"),  # Place
            self.make_pose(0.0, 0.0, 1.0, "410_ur5e_base_link"),   # Home
        ]

    def make_pose(self, x, y, z, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
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
            
        # Select poses based on current robot
        if self.current_robot == "409":
            poses = self.poses_409
            publisher = self.publisher_409
        else:
            poses = self.poses_410
            publisher = self.publisher_410
            
        if self.sequence_step >= len(poses):
            # Switch to the other robot or complete
            if self.current_robot == "409":
                self.current_robot = "410"
                self.sequence_step = 0
                self.get_logger().info("Switching to robot 410...")
                return
            else:
                self.get_logger().info("Both robot sequences complete!")
                self.destroy_node()
                return
                
        # Publish the next pose
        self.get_logger().info(f"Publishing pose {self.sequence_step + 1} for robot {self.current_robot}")
        publisher.publish(poses[self.sequence_step])
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
