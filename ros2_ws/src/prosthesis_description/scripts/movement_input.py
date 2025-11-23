#!/usr/bin/env python3

# Node that sends a joint trajectory goal to a ROS2 action server (which is read by joint_trajectory_controller)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryClient(Node):

    def __init__(self):
        super().__init__('trajectory_client')

        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def send_goal(self):
        self._client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = [
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
            "joint_thumb_ip"
        ]

        point = JointTrajectoryPoint()
        point.positions = [
            0.2,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            -0.5,
            -0.5,
            -0.5
        ]

        point.time_from_start.sec = 2

        goal_msg.trajectory.points.append(point)

        self.get_logger().info("Sending trajectory...")
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)


def main():
    rclpy.init()
    node = TrajectoryClient()
    node.send_goal()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
