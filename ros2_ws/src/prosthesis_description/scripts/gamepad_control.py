#!/usr/bin/env python3
"""
Gamepad/Joystick controller node for the prosthesis arm
Publishes to /arm_mvmt_goals and /pose_goals topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
# see joy package documentation at https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html
from std_msgs.msg import Float32MultiArray, Int8


class GamepadControlNode(Node):
    def __init__(self):
        super().__init__('gamepad_control')
        
        # Subscribers
        self.joy_subscription_ = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10
        )
        
        # Publishers
        self.arm_movement_publisher_ = self.create_publisher(
            Float32MultiArray, 'arm_mvmt_goals', 10
        )
        self.pose_publisher_ = self.create_publisher(
            Int8, 'pose_goals', 10
        )
        
        self.base_step = 0.03
        self.get_logger().info("Gamepad control node initialized")
        self.print_help()
    
    def print_help(self):
        """Print gamepad button mapping"""
        help_text = """
        ╔════════════════════════════════════════════════════════╗
        ║        PROSTHESIS GAMEPAD CONTROL                      ║
        ╠════════════════════════════════════════════════════════╣
        ║  LEFT STICK:                                           ║
        ║    Axis 0 (X) → X-axis movement (forward/backward)     ║
        ║    Axis 1 (Y) → Y-axis movement (left/right)           ║
        ║                                                        ║
        ║  RIGHT STICK:                                          ║
        ║    Axis 3 (X) → Z-axis movement (up/down)              ║
        ║                                                        ║
        ║  BUTTONS:                                              ║
        ║    A (0)     → Open hand                               ║
        ║    B (1)     → Close hand                              ║
        ║    X (2)     → Wrist right                             ║
        ║    Y (3)     → Wrist left                              ║
        ╚════════════════════════════════════════════════════════╝
        """
        self.get_logger().info(help_text)
    
    def joy_callback(self, msg):
        """Handle joystick input"""
        
        if msg.axes[0]:  # Forward/Backward
            movement_msg = Float32MultiArray()
            movement_msg.data = [1, msg.axes[0]* self.base_step]
            self.arm_movement_publisher_.publish(movement_msg)
            self.get_logger().info(f"Sent moving goal of {msg.axes[0]* self.base_step:.3f} m along x axis")
        
        if msg.axes[1]:  # Left/Right
            movement_msg = Float32MultiArray()
            movement_msg.data = [0, msg.axes[1] * self.base_step]
            self.arm_movement_publisher_.publish(movement_msg)
            self.get_logger().info(f"Sent moving goal of {msg.axes[1]* self.base_step:.3f} m along y axis")

        if msg.axes[3]: # Up/Down
            movement_msg = Float32MultiArray()
            movement_msg.data = [2, msg.axes[3] * self.base_step]
            self.arm_movement_publisher_.publish(movement_msg)
            self.get_logger().info(f"Sent moving goal of {msg.axes[3]* self.base_step:.3f} m along z axis")

        if msg.buttons[0]:  # A button → Open hand
            pose_msg = Int8()
            pose_msg.data = 0
            self.pose_publisher_.publish(pose_msg)
            self.get_logger().info(f"Sent pose number {pose_msg} goal")
        elif msg.buttons[1]:  # B button → Close hand
            pose_msg = Int8()
            pose_msg.data = 1
            self.pose_publisher_.publish(pose_msg)
            self.get_logger().info(f"Sent pose number {pose_msg} goal")
        elif msg.buttons[2]:  # X button → Wrist right
            pose_msg = Int8()
            pose_msg.data = 3
            self.pose_publisher_.publish(pose_msg)
            self.get_logger().info(f"Sent pose number {pose_msg} goal")
        elif msg.buttons[3]:  # Y button → Wrist left
            pose_msg = Int8()
            pose_msg.data = 4
            self.pose_publisher_.publish(pose_msg)
            self.get_logger().info(f"Sent pose number {pose_msg} goal")
        '''elif msg.buttons[10]:  # R button → Pinch
            pose_msg = Int8()
            pose_msg.data = 2
            self.pose_publisher_.publish(pose_msg)
            self.get_logger().info(f"Sent pose number {pose_msg} goal")'''

def main():
    rclpy.init()
    node = GamepadControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()