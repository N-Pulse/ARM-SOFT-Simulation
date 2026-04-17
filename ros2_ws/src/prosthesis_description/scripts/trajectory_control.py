#!/usr/bin/env python3

"""
Trajectory control node (handling the fingers dynamic) which subscribes to /trajectory_control and sends goals to the trajectory controller
"""
import rclpy
from rclpy.node import Node
#from launch import LaunchDescriptionEntity
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryControlNode(Node):
    
    def __init__(self):
        super().__init__('trajectory_control')
        
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
        
        self.joint_names = [
            "joint_world_to_base_x",
            "joint_base_x_to_base_y",
            "joint_base_y_to_base_z",
            "joint_wrist_to_palm",
            "joint_index_1",
            "joint_index_2",
            "joint_index_3",
            "joint_index_4",
            "joint_middle_1",
            "joint_middle_2",
            "joint_middle_3",
            "joint_middle_4",
            "joint_ring_1",
            "joint_ring_2",
            "joint_ring_3",
            "joint_ring_4",
            "joint_little_1",
            "joint_little_2",
            "joint_little_3",
            "joint_little_4",
            "joint_thumb_1",
            "joint_thumb_2",
            "joint_thumb_3"
        ]
        self.nb_joints = len(self.joint_names)
        self.max_positions = [1.57] * self.nb_joints # max. position for each joint (in radian)
        self.current_positions = [0.0] * self.nb_joints

        self.get_logger().info("Trajectory_control node initialized")


    def send_trajectory(self, positions, duration=0.1, goal_name="Goal"):
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
        
    def move_joint(self, joint_index, moving_percentage):
        """Update a joint position and send it"""
        self.current_positions[joint_index] += moving_percentage * self.max_positions[joint_index]
        self.send_trajectory(self.current_positions, duration=0.01)
        self.get_logger().info(f"Moving {self.joint_names[joint_index]}")

    def move_finger(self, finger_name, moving_percentage):
        """Update a finger position and send it"""
        if finger_name not in ["index", "middle", "ring", "little", "thumb"]:
            raise ValueError("The name of the finger must be index, middle, ring, little or thumb")

        if finger_name == "index":
            for i in range (4,7):
                self.move_joint(i, moving_percentage)

        if finger_name == "middle":
            for i in range (8,11):
                self.move_joint(i, moving_percentage)

        if finger_name == "ring":
            for i in range (12,15):
                self.move_joint(i, moving_percentage)

        if finger_name == "little":
            for i in range (16,19):
                self.move_joint(i, moving_percentage)

        if finger_name == "thumb":
            for i in range (20,22):
                self.move_joint(i, moving_percentage)

    #----------PREDEFINDED HAND POSES-------------------------
    def hand_pose_0(self):
        """open hand"""
        for i in ["index", "middle", "ring", "little", "thumb"]:
            self.move_finger(i,0)      

    def hand_pose_1(self):
        """close hand"""
        for i in ["index", "middle", "ring", "little", "thumb"]:
            self.move_finger(i,100)
    
    def hand_pose_2(self):
        """pinch"""
        for i in ["index", "thumb"]:
            self.move_finger(i,100) 
        for i in ["middle", "ring", "little"]:
            self.move_finger(i,0)
    
    def hand_pose_3(self):
        """rotate wrist right"""
        self.move_joint(3, 100)

    def hand_pose_4(self):
        """rotate wrist left"""
        self.move_joint(3, -100)
    #------------------------------------------------------

    def handle_arm_mvmt_goals(self, msg):
        joint_index = int(msg.data[0])
        moving_percentage = msg.data[1]*100
        self.move_joint(joint_index, moving_percentage)

    def handle_pose_goals(self, msg):
        if msg.data == 0 :
            self.hand_pose_0()
        if msg.data == 1 :
            self.hand_pose_1()
        if msg.data == 2 :
            self.hand_pose_2()
        if msg.data == 3 :
            self.hand_pose_3()
        if msg.data == 4 :
            self.hand_pose_4()




def main():
    rclpy.init()
    node = TrajectoryControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
