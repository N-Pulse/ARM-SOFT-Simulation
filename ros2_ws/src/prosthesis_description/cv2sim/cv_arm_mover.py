#!/usr/bin/env python3

'''Moves the prosthesis near the object as if the user wanted to grab it based
on the received position and orientation of the objet'''

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

class CvArmMoverNode(Node):
    
    def __init__(self):
        super().__init__('cv_arm_mover')

        self.arm_moved = False

        self.cv_model_subscription_ = self.create_subscription(
            Float64MultiArray, '/cv/model', self.cv_model_callback, 10
        )

        self.arm_movement_publisher_ = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        self.get_logger().info("cv_arm_mover node initialized")


    def cv_model_callback(self, msg):
        
        x = msg.data[1] #x
        y = msg.data[2] #y
        z = msg.data[3] #z
        r = msg.data[4] #roll
        p = msg.data[5] #pitch
        y = msg.data[6] #yaw

        trajectory = JointTrajectory()
        trajectory.joint_names.append('joint_base_x')
        trajectory.joint_names.append('joint_base_y')
        trajectory.joint_names.append('joint_base_z')
        trajectory.joint_names.append('joint_base_roll')
        trajectory.joint_names.append('joint_base_pitch')
        trajectory.joint_names.append('joint_base_yaw')
        point = JointTrajectoryPoint()
        point.positions.append(x)
        point.positions.append(y)
        point.positions.append(z)
        point.positions.append(r)
        point.positions.append(p)
        point.positions.append(y)
        trajectory.points.append(point)


        if self.arm_moved:
            self.get_logger().info('Arm already moved')
            return
        else:
            self.arm_movement_publisher_.publish(trajectory)
            self.arm_moved = True
            self.get_logger().info(
                f'Moving arm at ({x}, {y}, {z}) '
                f'with orientation (R={r:.3f}, P={p:.3f}, Y={y:.3f})'
            )

def main():
    rclpy.init()
    node = CvArmMoverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()