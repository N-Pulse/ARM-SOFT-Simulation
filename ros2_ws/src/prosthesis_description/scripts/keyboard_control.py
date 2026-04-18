#!/usr/bin/env python3

"""
Keyboard control node for the prosthesis arm base 
which publishes to /arm_mvmt_goals topic or /pose_goals topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
import time
import sys
import threading


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
        
        self.arm_movement_publisher_ = self.create_publisher(Float32MultiArray, 'arm_mvmt_goals', 10)
        self.pose_publisher_ = self.create_publisher(Int8, 'pose_goals', 10)

        self.base_step = 0.001  # meters for base joints
        
        self.get_logger().info("Keyboard control node initialized")
        self.print_help()
    

    
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
        ║    1    → Open hand                                    ║
        ║    2    → Close hand                                   ║
        ║    3    → Pinch                                        ║
        ║    4    → Wrist rotated 90° to the left                ║
        ║    5    → Wrist rotated 90° to the right               ║
        ║                                                        ║
        ║                                                        ║
        ║  OTHER:                                                ║
        ║    H    → Show this help                               ║
        ║    ESC  → Exit                                         ║
        ╚════════════════════════════════════════════════════════╝
        """
        self.get_logger().info(help_text)
    
    
    def handle_input(self, key):
        """Handle keyboard input"""
        if key == 'q' or key == 'Q':
            msg = Float32MultiArray()
            msg.data = [0, -self.base_step]
            self.arm_movement_publisher_.publish(msg)
            self.get_logger().info(f"Sent moving goal of {-self.base_step:.3f} m along x axis")
        elif key == 'e' or key == 'E':
            msg = Float32MultiArray()
            msg.data = [0, self.base_step]
            self.arm_movement_publisher_.publish(msg)
            self.get_logger().debug(f"Sent moving goal of {self.base_step:.3f} m along x axis")
        
        elif key == 'a' or key == 'A':
            msg = Float32MultiArray()
            msg.data = [1, -self.base_step]
            self.arm_movement_publisher_.publish(msg)
            self.get_logger().info(f"Sent moving goal of {self.base_step:.3f} m along y axis")
        elif key == 'd' or key == 'D':
            msg = Float32MultiArray()
            msg.data = [1, self.base_step]
            self.arm_movement_publisher_.publish(msg)
            self.get_logger().info(f"Sent moving goal of {self.base_step:.3f} m along y axis")
        
        elif key == 'w' or key == 'W':
            msg = Float32MultiArray()
            msg.data = [2, self.base_step]
            self.arm_movement_publisher_.publish(msg)
            self.get_logger().info(f"Sent moving goal of {self.base_step:.3f} m along z axis")
        elif key == 's' or key == 'S':
            msg = Float32MultiArray()
            msg.data = [2, -self.base_step]
            self.arm_movement_publisher_.publish(msg)
            self.get_logger().info(f"Sent moving goal of {-self.base_step:.3f} m along z axis")
        
        elif key in ['0', '1', '2', '3', '4', '5', '6']:
            pose_num = int(key)
            msg = Int8()
            msg.data = pose_num
            self.pose_publisher_.publish(msg)
            self.get_logger().info(f"Sent pose number {pose_num} goal")
        
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
