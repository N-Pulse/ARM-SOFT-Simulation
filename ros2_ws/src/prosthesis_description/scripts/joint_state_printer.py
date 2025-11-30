#!/usr/bin/env python3

# A simple node that subscribes to /joint_states and prints joint information

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class JointStatePrinter(Node):
    def __init__(self):
        super().__init__('joint_state_printer')

        # last received JointState message
        self.last_msg = None

        # FSR values storage
        self.fsr_values = {
            'index': None,
            'middle': None,
            'ring': None,
            'little': None,
            'thumb': None
        }

        # subscription to joint states
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.cb, 10
        )

        # subscriptions to FSR topics
        for finger in self.fsr_values.keys():
            self.create_subscription(
                Float32, f'/fsr/{finger}', 
                lambda msg, f=finger: self.fsr_cb(msg, f), 10
            )
        
        self.get_logger().info("JointStatePrinter initialized. Subscribing to FSR topics...")

        # timer prints every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_cb)

    # subscription callback
    def cb(self, msg):
        self.last_msg = msg

    # FSR callback
    def fsr_cb(self, msg, finger):
        self.fsr_values[finger] = msg.data
        self.get_logger().debug(f"Received FSR for {finger}: {msg.data}")

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

            # Get FSR value if this joint is related to a finger
            fsr_str = ""
            for finger in self.fsr_values.keys():
                if finger in name.lower():
                    fsr_val = self.fsr_values[finger]
                    fsr_str = f"fsr[{finger}]={fsr_val:.3f}" if fsr_val is not None else ""
                    break

            # printing joint information
            pos_str = f"{pos:.3f}" if pos is not None else "None"
            vel_str = f"{vel:.3f}" if vel is not None else "None"
            self.get_logger().info(
                f"ID={i}, name={name}, pos={pos_str}, vel={vel_str}, t={stamp}, {fsr_str}"
            )


def main():
    rclpy.init()
    rclpy.spin(JointStatePrinter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()