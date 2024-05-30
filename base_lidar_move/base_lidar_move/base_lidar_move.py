#!/usr/bin/env python3

# Librerías de ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

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

        # Crear el publicador
        self.angle_publisher = self.create_publisher(Float64MultiArray, "/joint_group_position_controller/commands", 10)
         
        self.angle_msg = Float64MultiArray()

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

        # Publicar el nuevo ángulo
        self.angle_msg.data = [self.new_joint_1_value]
        self.angle_publisher.publish(self.angle_msg)
        
        # Imprimir el valor actual del ángulo en grados
        self.get_logger().info(f"base_lidar_joint: {round(math.degrees(self.new_joint_1_value), 2)} °")

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
