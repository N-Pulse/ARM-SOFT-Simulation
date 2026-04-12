#!/usr/bin/env python3

"""
Trajectory control node (handling the fingers dynamic) which subscribes to /trajectory_control and sends goals to the trajectory controller
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from rclpy.action import ActionClient
import threading

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryControlNode(Node):
    def __init__(self):
        super().__init__('trajectory_control')
        
        self.arm_movement_subscription_ = self.create_subscription(
            Float32MultiArray,
            'arm_mvmt_goal',
            self.handle_arm_mvmt_goals)
        
        self.pose_subscription_ = self.create_subscription(
            Int8,
            'pose_goals',
            self.handle_pose_goals
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
            "joint_thumb_3",
            "joint_thumb_4"
        ]
        
        self.current_positions = [0.0] * 24 #24 joints
        
        # Predefined hand poses: use 'modifications' to specify which joints to override
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
                'modifications': {3: -1.571}
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

        self.get_logger().info("Trajectory_control node initialized")

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

    def move_joint(self, joint_index, delta):
        """Update a joint position and send it"""
        self.current_positions[joint_index] += delta
        self.send_trajectory(self.current_positions, duration=0.01)

    def move_finger(self, finger_name, folding_percentage):
        if finger_name not in ["index", "middle", "ring", "little", "thumb"]:
            raise ValueError("The name of the finger must be index, middle, ring, little or thumb")

        folding_angle = folding_percentage*1.57

        if finger_name == "index":
            self.move_joint(4, abs(self.current_positions[4]-folding_angle))
            self.move_joint(5, abs(self.current_positions[5]-folding_angle))
            self.move_joint(6, abs(self.current_positions[6]-folding_angle))
            self.move_joint(7, abs(self.current_positions[7]-folding_angle))

        if finger_name == "middle":
            self.move_joint(8, abs(self.current_positions[8]-folding_angle))
            self.move_joint(9, abs(self.current_positions[9]-folding_angle))
            self.move_joint(10, abs(self.current_positions[10]-folding_angle))
            self.move_joint(11, abs(self.current_positions[11]-folding_angle))

        if finger_name == "ring":
            self.move_joint(12, abs(self.current_positions[12]-folding_angle))
            self.move_joint(13, abs(self.current_positions[13]-folding_angle))
            self.move_joint(14, abs(self.current_positions[14]-folding_angle))
            self.move_joint(15, abs(self.current_positions[15]-folding_angle))

        if finger_name == "little":
            self.move_joint(16, abs(self.current_positions[16]-folding_angle))
            self.move_joint(17, abs(self.current_positions[17]-folding_angle))
            self.move_joint(18, abs(self.current_positions[18]-folding_angle))
            self.move_joint(19, abs(self.current_positions[19]-folding_angle))

        if finger_name == "thumb":
            self.move_joint(20, abs(self.current_positions[20]-folding_angle))
            self.move_joint(21, abs(self.current_positions[21]-folding_angle))
            self.move_joint(22, abs(self.current_positions[22]-folding_angle))
            self.move_joint(23, abs(self.current_positions[23]-folding_angle))

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
    

    def handle_arm_mvmt_goals(self, msg):

        joint_name = self.joint_names[msg[0]]
        delta = msg[1]
        self.move_joint(joint_name, delta)

    def handle_pose_goals(self, msg):

        if msg == 1 :
            self.move_finger("index",100)
        if msg == 2 :
            self.move_joint(self.joint_names[3], 1.57)
        
        #To be continued


def main():
    rclpy.init()
    node = TrajectoryControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
