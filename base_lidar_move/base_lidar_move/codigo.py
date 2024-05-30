import rclpy
from rclpy.node import Node
import serial
import time

class SerialCommanderNode(Node):
    def __init__(self):
        super().__init__('serial_commander')
        self.serial_port = '/dev/ttyACM0'  # Reemplaza con tu puerto serial
        self.baud_rate = 9600
        self.ser = serial.Serial(self.serial_port, self.baud_rate)
        time.sleep(2)  # Espera a que se establezca la conexión serial
        self.command_counter = 0
        self.create_timer(1.0, self.send_serial_commands)

    def send_serial_commands(self):
        # Envía el comando -142 diez veces
        for _ in range(10):
            self.ser.write(b'-142\n')
            time.sleep(1)  # Espera 1 segundo entre comandos
        
        # Envía el comando 1420 una vez
        self.ser.write(b'1420\n')
        time.sleep(10)  # Espera 10 segundos antes de reiniciar el ciclo

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
