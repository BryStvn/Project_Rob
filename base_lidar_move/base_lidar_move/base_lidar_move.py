#!/usr/bin/env python3

# Librerías de ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Librerías de Python
import math

class JointAnglePositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_group_position_publisher')

        # Definir ángulos iniciales y finales en radianes
        self.joint_1_initial_angle = math.radians(-25)
        self.joint_1_final_angle = math.radians(25)

        # Inicializar el valor del ángulo de la articulación y el incremento
        self.new_joint_1_value = self.joint_1_initial_angle
        self.increment_in_joint = math.radians(5)  # Incremento de 5 grados en radianes

        # Estado del movimiento: True para avance, False para retroceso
        self.is_advancing = True

        # Crear el publicador para Gazebo
        self.angle_publisher = self.create_publisher(Float64MultiArray, "/joint_group_position_controller/commands", 10)

        # Crear el publicador para RViz
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)

        # Crear el broadcaster de TF
        self.tf_broadcaster = TransformBroadcaster(self)
         
        self.angle_msg = Float64MultiArray()

        # Crear el mensaje de estado de articulación
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ["base_lidar_joint"]

        # Crear un temporizador que llame a la función publish_interpolation_trajectory cada 1 segundo para el avance
        self.interpolation_timer = self.create_timer(1.0, self.publish_interpolation_trajectory)
    
    def publish_interpolation_trajectory(self):
        # Verificar el estado de movimiento
        if self.is_advancing:
            # Incrementar el valor del ángulo
            self.new_joint_1_value += self.increment_in_joint

            # Cambiar la dirección del incremento cuando se alcanza el límite superior
            if self.new_joint_1_value >= self.joint_1_final_angle:
                self.new_joint_1_value = self.joint_1_final_angle
                self.is_advancing = False
                # Cambiar el temporizador a un intervalo más corto para el retroceso
                self.interpolation_timer.cancel()
                self.interpolation_timer = self.create_timer(0.1, self.publish_interpolation_trajectory)

        else:
            # Decrementar el valor del ángulo continuamente
            self.new_joint_1_value -= 0.01  # Incremento continuo

            # Cambiar el estado cuando se alcanza el límite inferior
            if self.new_joint_1_value <= self.joint_1_initial_angle:
                self.new_joint_1_value = self.joint_1_initial_angle
                self.is_advancing = True
                # Cambiar el temporizador a 1 segundo para el avance
                self.interpolation_timer.cancel()
                self.interpolation_timer = self.create_timer(1.0, self.publish_interpolation_trajectory)

        # Publicar el nuevo ángulo para Gazebo
        self.angle_msg.data = [self.new_joint_1_value]
        self.angle_publisher.publish(self.angle_msg)

        # Publicar el nuevo estado de la articulación para RViz
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = [self.new_joint_1_value]
        self.joint_state_publisher.publish(self.joint_state_msg)
        
        # Publicar la transformación para TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Cambia esto al frame adecuado
        t.child_frame_id = 'base_lidar_joint'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.new_joint_1_value)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # Imprimir el valor actual del ángulo en grados
        self.get_logger().info(f"base_lidar_joint: {round(math.degrees(self.new_joint_1_value), 2)} °")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    joint_angle_position_publisher_node = JointAnglePositionPublisher()
    try:
        rclpy.spin(joint_angle_position_publisher_node)
    except KeyboardInterrupt:
        joint_angle_position_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
