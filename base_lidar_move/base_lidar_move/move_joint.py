#!/usr/bin/env python3
#Librerias de ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
#Librerias de Python
import math

class JointAnglePositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_group_position_publisher')

        self.joint_1_initial_angle = math.radians(-45)
        self.joint_1_final_angle = math.radians(45)

        self.new_joint_1_value = 0.0
        self.increment_in_joint = 0.01  # Incremento inicial positivo

        self.angle_publisher = self.create_publisher(Float64MultiArray, "/joint_group_position_controller/commands", 10)
         
        self.angle_msg = Float64MultiArray()
        self.interpolation_timer = self.create_timer(0.1, self.publish_interpolation_trajectory)
    
    def publish_interpolation_trajectory(self):
        self.new_joint_1_value += self.increment_in_joint

        # Cambia la dirección del incremento cuando alcanza los límites
        if self.new_joint_1_value >= self.joint_1_final_angle or self.new_joint_1_value <= self.joint_1_initial_angle:
            self.increment_in_joint = -self.increment_in_joint

        self.angle_msg.data = [self.new_joint_1_value]
        self.angle_publisher.publish(self.angle_msg)
        
        print("\nCurrent Value in Joints:")
        print(f"base_lidar_joint: {round(math.degrees(self.new_joint_1_value), 2)} °")

def main(args=None):
    rclpy.init(args=args)
    joint_angle_position_publisher_node = JointAnglePositionPublisher()
    try:
        rclpy.spin(joint_angle_position_publisher_node)
    except KeyboardInterrupt:
        joint_angle_position_publisher_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()