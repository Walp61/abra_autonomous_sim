import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, Float32
from abra_interfaces.msg import AbraVehicleControl

CONTROL_TOPIC = "/vehicle_control"
PUBLISH_RATE = 10 #hz

SPEED_LIMIT = 25 #km/h

class JoyNode(Node):
    def __init__(self):
        super().__init__("abra_joy_node")
        self.joystick_subscriber = self.create_subscription(Joy, "/joy", self.joy_callback, 1)
        self.publisher = self.create_publisher(AbraVehicleControl, CONTROL_TOPIC, 1)
        self.current_data = Joy()
        self.current_velocity = 0
        self.current_steering_angle = 0
        self.pub_data = AbraVehicleControl()

        self.timer_period = 1.0 / PUBLISH_RATE  # seconds
        self.main_timer = self.create_timer(self.timer_period, self.main)

    def joy_callback(self, data):
        self.current_data = data

    def main(self):
        axes = self.current_data.axes
        buttons = self.current_data.buttons

        print(self.current_steering_angle)

        if buttons or axes:

            if buttons[3] == 1:
                if self.current_velocity < SPEED_LIMIT:
                    self.current_velocity += 1

            elif buttons[1] == 1:
                if self.current_velocity > -SPEED_LIMIT:
                    self.current_velocity -= 1

            if axes[4] == -1 and self.current_steering_angle < 33:
                self.current_steering_angle += 3
            elif axes[4] == 1 and self.current_steering_angle > -33:
                self.current_steering_angle -= 3

        if buttons[5] == 1:
            self.pub_data.linear_speed = self.current_velocity
        else:
            self.pub_data.linear_speed = 0
        
        self.pub_data.steering_angle = self.current_steering_angle
        self.publisher.publish(self.pub_data)


def main(args=None):
    rclpy.init(args=args)
    node = JoyNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

