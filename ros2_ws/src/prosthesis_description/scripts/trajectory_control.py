#!/usr/bin/env python3

"""
Trajectory control node (handling the fingers dynamic) which subscribes to /trajectory_control and sends goals to the trajectory controller
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryControlNode(Node):
    
    def __init__(self):
        super().__init__('trajectory_control')

        self.last_joint_states = None
        self.joint_states_subscription_ = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )
        
        self.arm_movement_subscription_ = self.create_subscription(
            Float32MultiArray,
            'arm_mvmt_goals',
            self.handle_arm_mvmt_goals,
            10
        )
        
        self.pose_subscription_ = self.create_subscription(
            Int8,
            'pose_goals',
            self.handle_pose_goals,
            10
        )
        
        # Action client for trajectory control
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info("Trajectory_control node initialized")

    def joint_states_callback(self, msg):
        self.last_joint_states = msg

    def send_trajectory(self, positions, duration=0.1, goal_name="Goal"):
        """Send a joint trajectory goal to the controller"""
        try:
            self._client.wait_for_server(timeout_sec=1.0)
        except Exception as e:
            self.get_logger().warning(f"Trajectory server not available: {e}")
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.last_joint_states.name
        
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
        
    def move_joint(self, joint_index, delta):
        """Update a joint position and send it"""
        if self.last_joint_states is None:
            self.get_logger().warn("No joint states received yet")
            return

        new_position = self.last_joint_states.position[joint_index] + delta
        
        new_positions = self.last_joint_states.position #make a copy
        new_positions[joint_index] = new_position
        self.send_trajectory(new_positions, duration=0.1)
        self.get_logger().info(f"Moving {self.last_joint_states.name[joint_index]} to {new_position:.3f}")

    def move_finger(self, finger_name, delta):
        """Update a finger position and send it"""
        if finger_name not in ["index", "middle", "ring", "little", "thumb"]:
            raise ValueError("The name of the finger must be index, middle, ring, little or thumb")

        finger_ranges = {
            "index": range(4, 7),
            "middle": range(7, 10),
            "ring": range(10, 13),
            "little": range(13, 16),
            "thumb": range(16, 19)
        }
        
        for i in finger_ranges[finger_name]:
            self.move_joint(i, delta)

    #----------PREDEFINDED HAND POSES-------------------------
    def hand_pose_0(self):
        """open hand"""
        for finger in ["index", "middle", "ring", "little", "thumb"]:
            self.move_finger(finger, -1)      

    def hand_pose_1(self):
        """close hand"""
        for finger in ["index", "middle", "ring", "little", "thumb"]:
            self.move_finger(finger, 1)
    
    def hand_pose_2(self):
        """pinch"""
        for finger in ["index", "thumb"]:
            self.move_finger(finger, 1) 
        for finger in ["middle", "ring", "little"]:
            self.move_finger(finger, -1)

    def hand_pose_3(self):
        """rotate wrist right"""
        self.move_joint(3, 1)

    def hand_pose_4(self):
        """rotate wrist left"""
        self.move_joint(3, -1)
    #------------------------------------------------------

    def handle_arm_mvmt_goals(self, msg):
        joint_index = int(msg.data[0])
        delta = msg.data[1]
        self.move_joint(joint_index, delta)

    def handle_pose_goals(self, msg):
        poses = {
            0: self.hand_pose_0,
            1: self.hand_pose_1,
            2: self.hand_pose_2,
            3: self.hand_pose_3,
            4: self.hand_pose_4
        }
        if msg.data in poses:
            poses[msg.data]()

def main():
    rclpy.init()
    node = TrajectoryControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
