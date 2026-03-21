#!/usr/bin/env python3

# Node that sends sequential joint trajectory goals to a ROS2 action server

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

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
        
        # Define predefined goals (12 joint positions each)
        # Goal 1: Neutral position (all joints at 0)
        self.goal_1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Goal 2: Wrist rotated at 90 degrees, fingers neutral
        self.goal_2 = [1.571, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Goal 3: All fingers at 60 degrees (1.047 rad)
        self.goal_3 = [1.047, 1.047, 1.047, 1.047, 1.047, 1.047, 1.047, 1.047, 1.047, 1.047, 1.047, 1.047]
        
        # List of all goals to execute sequentially
        self.goals = [
            ("Goal 1: Neutral (all 0)", self.goal_1, 3),
            ("Goal 2: Wrist rotated at 90 degrees (1.571 rad)", self.goal_2, 3),
            ("Goal 3: All fingers joints rotated at 60 degrees (1.047 rad)", self.goal_3, 3),
        ]

    def send_goal(self, positions, duration, goal_name):
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
        point.positions = positions
        point.time_from_start.sec = duration

        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Sending {goal_name}...")
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        # Wait for the movement to complete before sending next goal
        time.sleep(duration + 1)

    def execute_all_goals(self):
        """ Execute all predefined goals sequentially """
        for goal_name, positions, duration in self.goals:
            self.get_logger().info(f"\n=== Executing {goal_name} ===")
            self.send_goal(positions, duration, goal_name)
            self.get_logger().info(f"✓ {goal_name} completed\n")


def main():
    rclpy.init()
    node = TrajectoryClient()
    node.execute_all_goals()
    node.get_logger().info("All goals completed!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
