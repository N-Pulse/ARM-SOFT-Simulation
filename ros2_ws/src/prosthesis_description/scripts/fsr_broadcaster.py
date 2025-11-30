#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float32
import math
import random
from threading import Lock


class FSRBroadcaster(Node):
    """
    Simple FSR (Force Sensing Resistor) broadcaster node.
    Subscribes to wrench topics from force-torque sensors and publishes normalized FSR values.
    """
    
    def __init__(self):
        super().__init__('fsr_broadcaster')
        
        # Fingers and their sensor configurations
        self.fingers = {
            'index': 'index_middle_link',
            'middle': 'middle_middle_link',
            'ring': 'ring_middle_link',
            'little': 'little_middle_link',
            'thumb': 'thumb_distal_link'
        }
        
        # FSR parameters
        self.noise_stddev = 0.02  # Gaussian noise
        self.max_force = 100.0    # Maximum force (N) for normalization
        self.force_values = {}
        self.lock = Lock()
        
        # Initialize force values
        for finger_name in self.fingers.keys():
            self.force_values[finger_name] = 0.0
        
        # Create subscriptions and publishers (use private names to avoid Node property collisions)
        self._wrench_subs = {}
        self._fsr_pubs = {}

        for finger_name, link_name in self.fingers.items():
            # Subscribe to contact topic
            contact_topic = f'/world/default/model/prosthesis/link/{link_name}/sensor/{finger_name}_fsr/contacts'
            
            self._wrench_subs[finger_name] = self.create_subscription(
                Contacts,
                contact_topic,
                lambda msg, fname=finger_name: self._contact_callback(msg, fname),
                10  # QoS depth
            )
            
            # Create publisher for FSR value
            fsr_topic = f'/fsr/{finger_name}'
            self._fsr_pubs[finger_name] = self.create_publisher(Float32, fsr_topic, 10)
        
        # Timer to publish FSR values at 100 Hz
        self.create_timer(0.01, self._publish_fsr_values)
        
        self.get_logger().info('FSR Broadcaster started - subscribing to contact topics')
    
    def _contact_callback(self, msg: Contacts, finger_name: str):
        """
        Callback to process contact messages and extract normal force magnitude.
        """
        total_force = 0.0
        
        # Sum up normal forces from all contact points
        for contact in msg.contacts:
            for normal in contact.normals:
                # Normal force vector magnitude
                force_mag = math.sqrt(normal.x**2 + normal.y**2 + normal.z**2)
                total_force += force_mag
        
        # Normalize to 0-1 range
        normalized = min(1.0, total_force / self.max_force)
        
        with self.lock:
            self.force_values[finger_name] = normalized
    
    def _publish_fsr_values(self):
        """
        Publish FSR values with added Gaussian noise and saturation.
        """
        with self.lock:
            for finger_name, force_val in self.force_values.items():
                # Add Gaussian noise using Box-Muller transform
                u1 = random.random()
                u2 = random.random()
                noise = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2) * self.noise_stddev
                
                noisy_value = force_val + noise
                
                # Clamp to [0, 1]
                fsr_value = max(0.0, min(1.0, noisy_value))
                
                # Publish
                msg = Float32()
                msg.data = float(fsr_value)
                self._fsr_pubs[finger_name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FSRBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
