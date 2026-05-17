#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import subprocess
import os
from ament_index_python.packages import get_package_share_directory


class CvModelSpawnerNode(Node):
    def __init__(self):
        super().__init__('cv_model_spawner_node')
        
        self.pkg_name = 'prosthesis_description'
        self.pkg_share = get_package_share_directory(self.pkg_name)
        self.world_name = 'prosthesis_world'
        
        self.model_spawned = False
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/cv/model',
            self.pose_callback,
            10
        )
        
        self.get_logger().info(f'Spawner node running. Waiting for published data')

    def pose_callback(self, msg):
        
        model = msg.data[0] # cube or cylinder
        x = msg.data[1] #x
        y = msg.data[2] #y
        z = msg.data[3] #z
        r = msg.data[4] #roll
        p = msg.data[5] #pitch
        y = msg.data[6] #yaw
        
        if model == 0 :
            model_name = 'cube'
        elif model == 1 :
            model_name = 'cylinder'
        else :
            self.get_logger().error('Model is not defined')

        self.spawn_model(model_name, x, y, z, r, p, y)

    def spawn_model(self, model_name, x, y, z, roll, pitch, yaw):
        """Spawn model into the running Gazebo world"""
        
        if self.model_spawned:
            self.get_logger().info('Model already spawned')
            return
        
        self.get_logger().info(
            f'Spawning {model_name} at ({x}, {y}, {z}) '
            f'with orientation (R={roll:.3f}, P={pitch:.3f}, Y={yaw:.3f})'
        )
        
        model_sdf_path = os.path.join(self.pkg_share, 'cv2sim', f'{model_name}.sdf')

        cmd = [
            'ros2', 'launch', 'ros_gz_sim', 'gz_spawn_model.launch.py',
            f'world:={self.world_name}',
            f'file:={model_sdf_path}',
            f'entity_name:={model_name}',
            f'x:={x}',
            f'y:={y}',
            f'z:={z}',
            f'R:={roll}',
            f'P:={pitch}',
            f'Y:={yaw}'
        ]
        
        try:
            subprocess.run(cmd, check=True)
            self.model_spawned = True
            self.get_logger().info(f'Model spawned successfully in world: {self.world_name}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Error spawning model: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CvModelSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

'''
For tests
ros2 topic pub --once /cv/model std_msgs/Float64MultiArray "{data: [0.0, 5.0, 5.0, 0.1, 0.0, 1.2, 0.0]}"
'''