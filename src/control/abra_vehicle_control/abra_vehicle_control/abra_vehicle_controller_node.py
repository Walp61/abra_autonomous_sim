import rclpy
from rclpy.node import Node
from abra_interfaces.msg import AbraVehicleControl
import serial
import time
import serial.tools.list_ports


MAX_LINEAR_CONTROL_SIGNAL = 4095
MIN_LINEAR_CONTROL_SIGNAL = 900
MAX_LINEAR_SPEED = 25.0

MAX_STEERING_CONTROL_SIGNAL = 480
MAX_STEERING_ANGLE = 33

FREQUENCY = 3.0

class VehicleController(Node):
    def __init__(self):
        super().__init__('abra_vehicle_controller_node')
        self.subscription = self.create_subscription(
            AbraVehicleControl,
            'vehicle_control',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.ser = self.find_arduino()
        self.current_linear_control_signal = 0
        self.current_steering_control_signal = 0
        self.timer = self.create_timer(1/FREQUENCY, self.main_callback)
        if self.ser is None:
            self.get_logger().error("Arduino not found!")
        else:
            self.get_logger().info("Arduino connected.")

    def find_arduino(self):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            try:
                ser = serial.Serial(port.device, 9600, timeout=2)
                time.sleep(2)  # Wait for Arduino to initialize
                
                # Send a test command to Arduino
                ser.write(b'?\n')
                time.sleep(1)
                
                # Check if Arduino responds with a known response
                if ser.in_waiting > 0:
                    response = ser.readline().decode().strip()
                    if response == "Arduino ready":  # Adjust this to match your Arduino's startup message
                        return ser
                ser.close()
            except:
                pass
        return None

    def clear_serial_buffer(self, ser):
        while ser.in_waiting > 0:
            ser.read(ser.in_waiting)

    def send_integers_to_arduino(self, integer1, integer2):
        if self.ser is None:
            self.get_logger().error("No Arduino connection.")
            return

        self.get_logger().info(f"Sending: {integer1}, {integer2}")
        self.ser.write(f"{integer1},{integer2}\n".encode())  # Send the integers as a comma-separated string
        
        # Clear the buffer before waiting for acknowledgment
        self.clear_serial_buffer(self.ser)
    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.linear_speed}, {msg.steering_angle}")
        self.current_linear_control_signal = self.calculate_linear_control_signal(msg.linear_speed)
        self.current_steering_control_signal = self.calculate_steering_control_signal(msg.steering_angle)
    
    def main_callback(self):
        self.send_integers_to_arduino(self.current_linear_control_signal, self.current_steering_control_signal)
    
    def calculate_linear_control_signal(self, speed):
        tangent = (MAX_LINEAR_CONTROL_SIGNAL - MIN_LINEAR_CONTROL_SIGNAL) / MAX_LINEAR_SPEED
        if speed > 0:
            return int((tangent * speed) + MIN_LINEAR_CONTROL_SIGNAL)
        elif speed < 0:
            return int((tangent * speed) - MIN_LINEAR_CONTROL_SIGNAL)
        else:
            return MIN_LINEAR_CONTROL_SIGNAL
    
    def calculate_steering_control_signal(self, steering_angle):
        tangent = (MAX_STEERING_CONTROL_SIGNAL / MAX_STEERING_ANGLE)
        return int(tangent * steering_angle)

def main(args=None):
    rclpy.init(args=args)
    vehicle_controller = VehicleController()
    rclpy.spin(vehicle_controller)
    vehicle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
