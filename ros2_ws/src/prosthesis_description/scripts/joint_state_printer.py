#!/usr/bin/env python3

# A simple node that subscribes to /joint_states and prints joint information

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePrinter(Node):
    def __init__(self):
        super().__init__('joint_state_printer')

        # last received JointState message
        self.last_msg = None

        # subscription
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.cb, 10
        )

        # timer prints every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_cb)

    # subscription callback
    def cb(self, msg):
        self.last_msg = msg

    # timer callback
    def timer_cb(self):
        if self.last_msg is None:
            return

        # computing timestamp
        msg = self.last_msg
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # looping through each joint name
        for i, name in enumerate(msg.name):
            # retrieving position and velocity if available
            pos = msg.position[i] if i < len(msg.position) else None
            vel = msg.velocity[i] if i < len(msg.velocity) else None

            # printing joint information
            self.get_logger().info(
                f"ID={i}, name={name}, pos={pos}, vel={vel}, t={stamp}"
            )


def main():
    rclpy.init()
    rclpy.spin(JointStatePrinter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()