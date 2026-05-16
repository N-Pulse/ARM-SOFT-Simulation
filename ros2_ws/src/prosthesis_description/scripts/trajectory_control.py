#!/usr/bin/env python3

"""
Trajectory control node (handling the fingers dynamic) which subscribes to /trajectory_control and sends goals to the trajectory controller
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
        
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.get_logger().info("Trajectory_control node initialized")

    def joint_states_callback(self, msg):
        self.last_joint_states = msg

    def send_trajectory(self, new_positions, goal_name="Goal"):
        """Publish trajectory directly to controller """
        try:
            traj = JointTrajectory()
            traj.joint_names = self.last_joint_states.name
            
            point = JointTrajectoryPoint()
            point.positions = list(new_positions)
            
            traj.points.append(point)
            self.trajectory_pub.publish(traj)
            self.get_logger().info(f"Published: {goal_name}")
            return True
        except Exception as e:
            self.get_logger().info(f"Could not publish trajectory: {e}")
            return False
    
    def send_batch_trajectory(self, joint_updates_dict, goal_name="Batch"):
        """Send multiple joint targets in one trajectory goal (parallel execution)"""
        if self.last_joint_states is None:
            self.get_logger().warn("No joint states received yet")
            return False
        
        new_positions = list(self.last_joint_states.position)
        for joint_idx, target_pos in joint_updates_dict.items():
            if 0 <= joint_idx < len(new_positions):
                new_positions[joint_idx] = target_pos
            else:
                self.get_logger().warn(f"Joint index {joint_idx} out of range")
        
        self.send_trajectory(new_positions, goal_name=goal_name)
        
    def move_joint(self, joint_index, delta):
        """Update a joint position and send it"""
        if self.last_joint_states is None:
            self.get_logger().warn("No joint states received yet")
            return

        new_position = self.last_joint_states.position[joint_index] + delta
        
        new_positions = self.last_joint_states.position
        new_positions[joint_index] = new_position
        self.send_trajectory(new_positions)

    def move_finger(self, finger_name, delta):
        """Update a finger position and send it"""
        if finger_name not in ["index", "middle", "ring", "little", "thumb_x", "thumb_y"]:
            raise ValueError("The name of the finger must be index, middle, ring, little or thumb_x or thumb_y")

        finger_ranges = {
            "thumb_x": [5],
            "thumb_y": range(6, 8),
            "index": range(9, 11),
            "middle": range(12, 14),
            "ring": range(15, 17),
            "little": range(18, 20)
        }
        
        for i in finger_ranges[finger_name]:
            self.move_joint(i, delta)

    #----------PREDEFINDED HAND POSES (BATCH/PARALLEL EXECUTION)-------------------------
    def hand_pose_0(self):
        """open hand - all fingers move in parallel"""
        joint_updates = {}
        # Index: joints 9, 10
        for i in range(9, 11):
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        # Middle: joints 12, 13
        for i in range(12, 14):
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        # Ring: joints 15, 16
        for i in range(15, 17):
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        # Little: joints 18, 19
        for i in range(18, 20):
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        # Thumb X: joint 5
        joint_updates[5] = self.last_joint_states.position[5] - 1.4
        # Thumb Y: joints 6, 7
        for i in range(6, 8):
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        
        self.send_batch_trajectory(joint_updates, goal_name="Hand Open")

    def hand_pose_1(self):
        """close hand - all fingers move in parallel"""
        joint_updates = {}
        # Index: joints 9, 10
        for i in range(9, 11):
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        # Middle: joints 12, 13
        for i in range(12, 14):
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        # Ring: joints 15, 16
        for i in range(15, 17):
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        # Little: joints 18, 19
        for i in range(18, 20):
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        # Thumb X: joint 5
        joint_updates[5] = self.last_joint_states.position[5] + 1.4
        
        self.send_batch_trajectory(joint_updates, goal_name="Hand Closed")
    
    def hand_pose_2(self):
        """pinch - index and thumb close, others open (parallel execution)"""
        joint_updates = {}
        # Index close: joints 9, 10
        for i in range(9, 11):
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        # Thumb Y close: joints 6, 7
        for i in range(6, 8):
            joint_updates[i] = self.last_joint_states.position[i] + 1.4
        # Middle open: joints 12, 13
        for i in range(12, 14):
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        # Ring open: joints 15, 16
        for i in range(15, 17):
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        # Little open: joints 18, 19
        for i in range(18, 20):
            joint_updates[i] = self.last_joint_states.position[i] - 1.4
        
        self.send_batch_trajectory(joint_updates, goal_name="Pinch")

    def hand_pose_3(self):
        """rotate wrist right"""
        joint_updates = {3: self.last_joint_states.position[3] + 1.57}
        self.send_batch_trajectory(joint_updates, goal_name="Wrist Right")

    def hand_pose_4(self):
        """rotate wrist left"""
        joint_updates = {3: self.last_joint_states.position[3] - 1.57}
        self.send_batch_trajectory(joint_updates, goal_name="Wrist Left")

    def hand_pose_5(self):
        """rotate wrist up"""
        joint_updates = {4: self.last_joint_states.position[4] + 1.57}
        self.send_batch_trajectory(joint_updates, goal_name="Wrist Up")

    def hand_pose_6(self):
        """rotate wrist down"""
        joint_updates = {4: self.last_joint_states.position[4] - 1.57}
        self.send_batch_trajectory(joint_updates, goal_name="Wrist Down")
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
