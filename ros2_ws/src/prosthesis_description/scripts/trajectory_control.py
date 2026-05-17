#!/usr/bin/env python3

"""
Trajectory control node (handling the fingers dynamic) which subscribes to /trajectory_control and sends goals to the trajectory controller
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryControlNode(Node):
    
    def __init__(self):
        super().__init__('trajectory_control')

        self.last_joint_states = None
        self.joint_states_subscription_ = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )
        
        self.arm_movement_subscription_ = self.create_subscription(
            JointTrajectory,
            'arm_delta_goals',
            self.handle_arm_delta_goals,
            10
        )
        
        self.pose_subscription_ = self.create_subscription(
            Int8,
            'pose_goals',
            self.handle_pose_goals,
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.get_logger().info("Trajectory_control node initialized")

    def joint_states_callback(self, msg):
        self.last_joint_states = msg
        
        self.wrist_indexes = []
        self.thumb_indexes = []
        self.index_indexes = []
        self.middle_indexes = []
        self.ring_indexes = []
        self.little_indexes = []
        for i in range(len(self.last_joint_states.name)) :
            if 'wrist' in self.last_joint_states.name[i]:
                self.wrist_indexes.append(i)
            if 'thumb' in self.last_joint_states.name[i]:
                self.thumb_indexes.append(i)
            if 'index' in self.last_joint_states.name[i]:
                self.index_indexes.append(i)
            if 'middle' in self.last_joint_states.name[i]:
                self.middle_indexes.append(i)
            if 'ring' in self.last_joint_states.name[i]:
                self.ring_indexes.append(i)
            if 'little' in self.last_joint_states.name[i]:
                self.little_indexes.append(i)

    def send_trajectory(self, trajectory : JointTrajectory, goal_name="Goal"):
        """Publish trajectory directly to controller """    
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info(f"Published: {goal_name}")
    
    def send_pose_trajectory(self, joint_updates_dict, goal_name="Pose"):
        """Send multiple joint targets in one trajectory goal (parallel execution)"""
        if self.last_joint_states is None:
            self.get_logger().warn("No joint states received yet")
        
        new_positions = list(self.last_joint_states.position)
        for joint_idx, target_pos in joint_updates_dict.items():
            if 0 <= joint_idx < len(new_positions):
                new_positions[joint_idx] = target_pos
            else:
                self.get_logger().error(f"Joint index {joint_idx} out of range")
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.last_joint_states.name
        point = JointTrajectoryPoint()
        point.positions = list(new_positions)
        point.time_from_start = Duration(sec=0, nanosec=0)
        trajectory.points.append(point)
        
        self.send_trajectory(trajectory, goal_name=goal_name)

    #----------PREDEFINDED HAND POSES (BATCH/PARALLEL EXECUTION)-------------------------
    def hand_pose_0(self):
        """open hand - all fingers move in parallel"""
        joint_updates = {}
        for i in self.index_indexes:
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        for i in self.middle_indexes:
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        for i in self.ring_indexes:
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        for i in self.little_indexes:
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        # Thumb X
        joint_updates[self.thumb_indexes[0]] = self.last_joint_states.position[5] - 1.4
        # Thumb Y
        for i in self.thumb_indexes[1::]:
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        
        self.send_pose_trajectory(joint_updates, goal_name="Hand Open")

    def hand_pose_1(self):
        """close hand - all fingers move in parallel"""
        joint_updates = {}
        for i in self.index_indexes:
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        for i in self.middle_indexes:
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        for i in self.ring_indexes:
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        for i in self.little_indexes:
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        # Thumb X
        joint_updates[self.thumb_indexes[0]] = self.last_joint_states.position[5] + 1.4
        
        self.send_pose_trajectory(joint_updates, goal_name="Hand Closed")
    
    def hand_pose_2(self):
        """pinch - index and thumb close, others open (parallel execution)"""
        joint_updates = {}
        for i in self.index_indexes:
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        # Thumb Y
        for i in self.thumb_indexes[1::]:
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        for i in self.middle_indexes:
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        for i in self.ring_indexes:
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        for i in self.little_indexes:
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        
        self.send_pose_trajectory(joint_updates, goal_name="Pinch")

    def hand_pose_3(self):
        """rotate wrist right"""
        index = self.wrist_indexes[0]
        joint_updates = {index: self.last_joint_states.position[index] + 1.57}
        self.send_pose_trajectory(joint_updates, goal_name="Wrist Right")

    def hand_pose_4(self):
        """rotate wrist left"""
        index = self.wrist_indexes[0]
        joint_updates = {index: self.last_joint_states.position[index] - 1.57}
        self.send_pose_trajectory(joint_updates, goal_name="Wrist Left")

    def hand_pose_5(self):
        """rotate wrist up"""
        index = self.wrist_indexes[1]
        joint_updates = {index: self.last_joint_states.position[index] + 1.57}
        self.send_pose_trajectory(joint_updates, goal_name="Wrist Up")

    def hand_pose_6(self):
        """rotate wrist down"""
        index = self.wrist_indexes[1]
        joint_updates = {index: self.last_joint_states.position[index] - 1.57}
        self.send_pose_trajectory(joint_updates, goal_name="Wrist Down")
    #------------------------------------------------------

    def handle_arm_delta_goals(self, msg):
        trajectory = JointTrajectory()
        trajectory.joint_names = msg.joint_names
        first_trajectory_point = JointTrajectoryPoint()
        msg_index = 0
        for joint_name in msg.joint_names :
            joint_states_index = self.last_joint_states.name.index(joint_name)
            last_position = self.last_joint_states.position[joint_states_index]
            delta = msg.points[0].positions[msg_index]
            new_position = last_position + delta
            first_trajectory_point.positions.append(new_position)
            msg_index += 1

        first_trajectory_point.time_from_start = Duration(sec=0, nanosec=0)
        trajectory.points.append(first_trajectory_point)
        self.send_trajectory(trajectory, 'Arm_delta')

    def handle_pose_goals(self, msg):
        poses = {
            0: self.hand_pose_0,
            1: self.hand_pose_1,
            2: self.hand_pose_2,
            3: self.hand_pose_3,
            4: self.hand_pose_4,
            5: self.hand_pose_5,
            6: self.hand_pose_6
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
