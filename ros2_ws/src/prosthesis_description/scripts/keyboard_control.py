#!/usr/bin/env python3

"""
Keyboard control node for the prosthesis arm base
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
import threading
import time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# implements cross-platform keyboard input to capture a
# single character without the user needing to press enter
# see https://gist.github.com/jasonrdsouza/1901709?permalink_comment_id=2734411
try:
    import os
    if os.name == 'nt': # check if running on Windows
        import msvcrt
        def getch():
            return msvcrt.getch().decode()
    else:
        import tty
        import termios
        def getch():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            if ord(ch) in [3, 4, 26, 27]: # handle ctrl+c
                sys.exit()    
            return ch
except ImportError:
    def getch():
        return 'a'  # fallback


class KeyboardControlNode(Node):
    
    def __init__(self):
        super().__init__('keyboard_control')
        
        # Action client for trajectory control
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Joint names: 3 base joints + 12 finger joints
        self.joint_names = [
            "joint_world_to_base_x",
            "joint_base_x_to_base_y",
            "joint_base_y_to_base_z",
            "joint_wrist_to_palm",
            "joint_index_mcp",
            "joint_index_pip",
            "joint_middle_mcp",
            "joint_middle_pip",
            "joint_ring_mcp",
            "joint_ring_pip",
            "joint_little_mcp",
            "joint_little_pip",
            "joint_thumb_cmc",
            "joint_thumb_mcp",
            "joint_thumb_pip"
        ]
        
        # Current joint positions (15 joints total)
        self.current_positions = [0.0] * 15
        
        # Movement speed / step size
        self.base_step = 0.01  # meters for base joints
        self.hand_step = 0.1   # radians for hand joints
        
        # Predefined hand poses: use 'modifications' to specify which joints to override
        # This is much cleaner than hardcoding all 15 positions for each pose
        self.hand_poses = {
            '1': {
                'name': 'Neutral (all 0)',
                'modifications': {1: 0., 2: 0., 3: 0., 4: 0., 5: 0., 6: 0., 7: 0., 8: 0., 9: 0., 10: 0., 11: 0., 12: 0., 13: 0., 14: 0.}
            },
            '2': {
                'name': 'All fingers to 0°',
                'modifications': {4: 0., 5: 0., 6: 0., 7: 0., 8: 0., 9: 0., 10: 0., 11: 0., 12: 0., 13: 0., 14: 0.}
            },
            '3': {
                'name': 'Wrist to 0°',
                'modifications': {3: 0.}
            },
            '4': {
                'name': 'Wrist rotated 90° to the left',
                'modifications': {3: -1.571}  # Only modify wrist (index 3)
            },
            '5': {
                'name': 'Wrist rotated 90° to the right',
                'modifications': {3: 1.571}
            },
            '6': {
                'name': 'All fingers flexed to 60°',
                'modifications': {4: 1.047, 5: 1.047, 6: 1.047, 7: 1.047, 8: 1.047, 9: 1.047, 10: 1.047, 11: 1.047, 12: 1.047, 13: 1.047, 14: 1.047}
            },
            '7': {
                'name': 'Fingers flexed + wrist left rotated 90°',
                'modifications': {3: -1.571, 4: 1.047, 5: 1.047, 6: 1.047, 7: 1.047, 8: 1.047, 9: 1.047, 10: 1.047, 11: 1.047, 12: 1.047, 13: 1.047, 14: 1.047}
            },
            '8': {
                'name': 'Fingers flexed + wrist right rotated 90°',
                'modifications': {3: 1.571, 4: 1.047, 5: 1.047, 6: 1.047, 7: 1.047, 8: 1.047, 9: 1.047, 10: 1.047, 11: 1.047, 12: 1.047, 13: 1.047, 14: 1.047}
            }
        }
        
        self.get_logger().info("Keyboard control node initialized")
        self.print_help()
    
    def pose_modifications(self, modifications):
        """Create a pose based on modifications to the current positions.
        
        Args:
            modifications (dict): {joint_index: value} pairs to override defaults
        
        Returns:
            list: Full position vector with specified joints modified
        """
        positions = self.current_positions # Start from neutral/zero
        for idx, value in modifications.items():
            positions[idx] = value
        return positions
    
    def print_help(self):
        """Print the help message"""
        help_text = """
        ╔════════════════════════════════════════════════════════╗
        ║        PROSTHESIS KEYBOARD CONTROL                     ║
        ╠════════════════════════════════════════════════════════╣
        ║  BASE LINK MOVEMENT:                                   ║
        ║    Q/E  → X-axis (forward/backward)                    ║
        ║    A/D  → Y-axis (left/right)                          ║
        ║    W/S  → Z-axis (up/down)                             ║
        ║                                                        ║
        ║  HAND CONTROL:                                         ║
        ║    1    → Neutral (all 0°)                             ║
        ║    2    → All fingers to 0°                            ║
        ║    3    → Wrist to 0°                                  ║
        ║    4    → Wrist rotated 90° to the left                ║
        ║    5    → Wrist rotated 90° to the right               ║
        ║    6    → All fingers flexed to 60°                    ║
        ║    7    → Fingers flexed + wrist left rotated 90°      ║
        ║    8    → Fingers flexed + wrist right rotated 90°     ║
        ║                                                        ║
        ║  OTHER:                                                ║
        ║    H    → Show this help                               ║
        ║    ESC  → Exit                                         ║
        ╚════════════════════════════════════════════════════════╝
        """
        self.get_logger().info(help_text)
    
    def send_trajectory(self, positions, duration=0.5, goal_name="Goal"):
        """Send a joint trajectory goal to the controller"""
        try:
            self._client.wait_for_server(timeout_sec=1.0)
        except Exception as e:
            self.get_logger().warning(f"Trajectory server not available: {e}")
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        
        goal_msg.trajectory.points.append(point)
        
        try:
            send_goal_future = self._client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=0.5)
            self.get_logger().debug(f"Sent: {goal_name}")
            return True
        except Exception as e:
            self.get_logger().debug(f"Could not send goal: {e}")
            return False
    
    def update_position(self, joint_index, delta):
        """Update a joint position and send it"""
        self.current_positions[joint_index] += delta
        self.send_trajectory(self.current_positions, duration=0.1)
    
    def handle_input(self, key):
        """Handle keyboard input"""
        if key == 'q' or key == 'Q':
            self.update_position(0, self.base_step)
            self.get_logger().info(f"X: {self.current_positions[0]:.2f}m")
        elif key == 'e' or key == 'E':
            self.update_position(0, -self.base_step)
            self.get_logger().info(f"X: {self.current_positions[0]:.2f}m")
        
        elif key == 'a' or key == 'A':
            self.update_position(1, -self.base_step)
            self.get_logger().info(f"Y: {self.current_positions[1]:.2f}m")
        elif key == 'd' or key == 'D':
            self.update_position(1, self.base_step)
            self.get_logger().info(f"Y: {self.current_positions[1]:.2f}m")
        
        elif key == 'w' or key == 'W':
            self.update_position(2, self.base_step)
            self.get_logger().info(f"Z: {self.current_positions[2]:.2f}m")
        elif key == 's' or key == 'S':
            self.update_position(2, -self.base_step)
            self.get_logger().info(f"Z: {self.current_positions[2]:.2f}m")
        
        elif key in self.hand_poses:
            pose = self.hand_poses[key]
            self.current_positions = self.pose_modifications(pose['modifications'])
            self.send_trajectory(self.current_positions, duration=1.0, goal_name=pose['name'])
            self.get_logger().info(f"Executing: {pose['name']}")
        
        elif key == 'h' or key == 'H':
            self.print_help()
        
        elif key == '\x1b':  # ESC key
            self.get_logger().info("Exiting keyboard control...")
            return False
        
        return True
    
    def keyboard_loop(self):
        """Main keyboard input loop"""
        self.get_logger().info("Waiting for 1 second, then starting keyboard input...")
        time.sleep(1)
        
        running = True
        while running and rclpy.ok():
            try:
                key = getch()
                if not self.handle_input(key):
                    running = False
            except KeyboardInterrupt:
                self.get_logger().info("Interrupted by user")
                running = False
            except Exception as e:
                self.get_logger().debug(f"Input error: {e}")
    
    def run(self):
        """Run the keyboard control node"""
        # Start keyboard input in a separate thread
        input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        input_thread.start()
        
        # Keep the ROS2 node spinning
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("Node interrupted")


def main():
    rclpy.init()
    node = KeyboardControlNode()
    
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
