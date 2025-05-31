#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, 'abra/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.key_thread = threading.Thread(target=self.key_capture_thread)
        self.key_thread.start()

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)

    def key_capture_thread(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                ch = sys.stdin.read(1)
                if ch == 'w':
                    self.linear_velocity += 0.1
                elif ch == 'a':
                    self.angular_velocity += 0.0872665  # 5 degrees in radians
                elif ch == 'd':
                    self.angular_velocity -= 0.0872665  # 5 degrees in radians
                elif ch == 's':
                    self.linear_velocity -= 0.1
                elif ch == ' ':
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.0
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    keyboard_controller = KeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
