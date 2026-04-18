#!/usr/bin/env python3

# subscribes to sensors topics and publishes current to /resulting_current

import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import JointState
#from sensor_msgs.msg import Imu
#from ros_gz_interfaces.msg import Contacts
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray

class SensorDataExtractor(Node):
    def __init__(self):
        super().__init__("sensor_data_extractor")

        '''
        self.imu_subscription_ = self.create_subscription(
            Imu,
            'prosthesis/sensor/palm_imu',
            self.extract_sensor_data(),
            10
        )
        
        self.fsr_values = {
            'wrist' : None,
            'index': None,
            'middle': None,
            'ring': None,
            'little': None,
            'thumb': None
        }

        for finger in self.fsr_values.keys():
            self.create_subscription(
                Contacts, f'/fsr/{finger}', 
                lambda msg, f=finger: self.extract_fsr_data(msg, f), 10
            )



        self.joint_state_subscription_ = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        '''
        self.torque_values = {
            'wrist' : None,
            'index': None,
            'middle': None,
            'ring': None,
            'little': None,
            'thumb': None
        } # should be one for each motor

        self.motor_torque_constant = 4/5

        for finger in self.torque_values.keys():
            self.create_subscription(
                WrenchStamped, f'prosthesis/sensor/{finger}_ft', 
                lambda msg, f=finger: self.extract_ft_data(msg, f), 10
            )
        
        self.resulting_current_publisher_ = self.create_publisher(
            Float32MultiArray,
            '/resulting_current',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Sensor data extractor node initialized")

    
    def joint_state_callback(self, msg):
        self.last_joint_state_msg = msg

    def extract_ft_data(self, msg, finger_name):
        self.torque_values[finger_name] = msg.data.wrench.torque
        self.get_logger().debug(f"Received torque for {finger_name}: {msg.data.wrench.torque}")
    
    def torque_to_current(self, torque_value):
        current_value = torque_value / self.motor_torque_constant
        return current_value
    
    def timer_callback(self):

        motors_resulting_current = Float32MultiArray()
        for finger in self.torque_values.keys():
            motors_resulting_current.data.append(
                self.torque_to_current(self, self.torque_values[finger])
            )
        self.resulting_current_publisher_.publish(motors_resulting_current)

def main():
    rclpy.init()
    node = SensorDataExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
