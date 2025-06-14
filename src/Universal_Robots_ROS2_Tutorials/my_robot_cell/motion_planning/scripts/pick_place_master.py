#!/usr/bin/env python3
# Coordinated Pick & Place Master with Gripper Placeholders
# Sequence: 410 picks -> 409 assists -> 410 places
# Maintainer: @dstreib

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import threading

class CoordinatedPickPlaceMaster(Node):
    def __init__(self):
        super().__init__('coordinated_pick_place_master')
        
        # Publishers for both robots
        self.publisher_409 = self.create_publisher(PoseStamped, 'pose_goal_409', 10)
        self.publisher_410 = self.create_publisher(PoseStamped, 'pose_goal_410', 10)
        
        # Gripper command publishers (placeholders for now)
        self.gripper_409 = self.create_publisher(String, 'gripper_409_command', 10)
        self.gripper_410 = self.create_publisher(String, 'gripper_410_command', 10)
        
        self.timer = self.create_timer(2.0, self.run_coordinated_sequence)
        self.sequence_step = 0
        self.executing = False

        # Define the coordinated sequence
        self.sequence_actions = [
            # Phase 1: Robot 410 approaches object
            {"action": "move", "robot": "410", "pose": [-0.15, -0.1, 0.25], "description": "410: Move to pre-pick position"},
            {"action": "move", "robot": "410", "pose": [-0.15, -0.1, 0.08], "description": "410: Move down to object"},
            {"action": "gripper", "robot": "410", "command": "close", "description": "410: GRIP OBJECT"},
            
            
            # Phase 2: Robot 409 assists/inspects
            {"action": "move", "robot": "409", "pose": [-0.05, -0.1, 0.25], "description": "409: Move to assist position"},
            {"action": "move", "robot": "409", "pose": [-0.05, -0.1, 0.12], "description": "409: Move down to assist"},
            {"action": "gripper", "robot": "409", "command": "open", "description": "409: OPEN GRIPPER (ready to assist)"},
            {"action": "wait", "duration": 1.0, "description": "409: Inspect/assist action"},
            {"action": "move", "robot": "409", "pose": [-0.05, -0.1, 0.25], "description": "409: Move back up"},
            {"action": "move", "robot": "409", "pose": [0.0, 0.0, 0.35], "description": "409: Move to home position"},
            
            # Phase 3: Robot 410 places object
            {"action": "move", "robot": "410", "pose": [-0.3, 0.1, 0.25], "description": "410: Move to pre-place position"},
            {"action": "move", "robot": "410", "pose": [-0.3, 0.1, 0.08], "description": "410: Move down to place"},
            {"action": "gripper", "robot": "410", "command": "open", "description": "410: RELEASE OBJECT"},
            {"action": "move", "robot": "410", "pose": [-0.3, 0.1, 0.25], "description": "410: Move up from place"},
            {"action": "move", "robot": "410", "pose": [0.0, 0.0, 0.35], "description": "410: Move to home position"},
        ]

    def make_pose(self, x, y, z, frame_id="table"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Facing down orientation
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose

    def send_gripper_command(self, robot, command):
        """Send gripper command (placeholder implementation)"""
        msg = String()
        msg.data = command
        
        if robot == "409":
            self.gripper_409.publish(msg)
        elif robot == "410":
            self.gripper_410.publish(msg)
            
        self.get_logger().info(f"🤖 GRIPPER {robot}: {command.upper()}")

    def send_motion_command(self, robot, pose_coords):
        """Send motion command to specified robot"""
        pose = self.make_pose(pose_coords[0], pose_coords[1], pose_coords[2])
        
        if robot == "409":
            self.publisher_409.publish(pose)
        elif robot == "410":
            self.publisher_410.publish(pose)
            
        self.get_logger().info(f"🎯 MOTION {robot}: Moving to [{pose_coords[0]:.2f}, {pose_coords[1]:.2f}, {pose_coords[2]:.2f}]")

    def run_coordinated_sequence(self):
        if self.sequence_step >= len(self.sequence_actions):
            self.get_logger().info("🎉 COORDINATED SEQUENCE COMPLETED!")
            return
            
        # Get current action
        action = self.sequence_actions[self.sequence_step]
        
        # Log the action description
        self.get_logger().info(f"📋 Step {self.sequence_step + 1}/{len(self.sequence_actions)}: {action['description']}")
        
        # Execute the action
        if action["action"] == "move":
            self.send_motion_command(action["robot"], action["pose"])
            
        elif action["action"] == "gripper":
            self.send_gripper_command(action["robot"], action["command"])
            
        elif action["action"] == "wait":
            self.get_logger().info(f"⏳ WAITING: {action['description']}")
            
        # Move to next step
        self.sequence_step += 1

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatedPickPlaceMaster()
    
    # Print sequence overview
    node.get_logger().info("🚀 STARTING COORDINATED PICK & PLACE SEQUENCE")
    node.get_logger().info("📝 Sequence Overview:")
    for i, action in enumerate(node.sequence_actions):
        node.get_logger().info(f"   {i+1:2d}. {action['description']}")
    node.get_logger().info("=" * 60)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
