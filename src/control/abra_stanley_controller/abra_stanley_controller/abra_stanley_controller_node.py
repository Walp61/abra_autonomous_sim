#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from abra_interfaces.msg import AbraPath, AbraState, AbraControllerState, AbraVehicleControl
from std_msgs.msg import String
import math
import time

#Control parameters:
k = 0.5 #Control gain
L = 1.42 #Wheelbase (meters)
goal_tolerance = 0.3 #(meters)

#Publisher parameters:
PUBLISH_FREQ = 10 #Publishing frequency.

class PathTrackingNode(Node):
    def __init__(self):
        super().__init__('abra_stanley_controller_node')
        self.subscription_state = self.create_subscription(
            AbraState,
            'abra/localization_state',
            self.listener_state_callback,
            10)
        self.subscription_path = self.create_subscription(
            AbraPath,
            'abra/current_path',
            self.listener_path_callback,
            10)
        self.subscription_controller_state = self.create_subscription(AbraControllerState, "abra/controller_state", self.listener_controller_state_callback,10)
        #self.publisher_ = self.create_publisher(AbraVehicleControl, '/vehicle_control', 1)
        self.publisher_ = self.create_publisher(Twist, "/abra/cmd_vel", 1)
        self.feedback_publisher = self.create_publisher(String, "abra/controller_feedback", 10)
        self.timer = self.create_timer(1/PUBLISH_FREQ, self.publish_steering_angle)
        self.state = None
        self.path = None
        self.target_idx = None
        self.last_idx = None
        self.id = -1
        self.controller_state = AbraControllerState()
        self.current_controller_feedback = ""
        self.ignition_flag = False

    def stanley_control(self, state, cx, cy, cyaw, last_target_idx):
        """
        Stanley steering control.

        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = self.calc_target_index(state, cx, cy)
        print(current_target_idx)
        print(last_target_idx)
        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = self.normalize_angle(cyaw[current_target_idx] - state.yaw)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(k * error_front_axle, state.v)
        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx

    def angle_mod(self, x, zero_2_2pi=False, degree=False):
        """
        Angle modulo operation
        Default angle modulo range is [-pi, pi)

        Parameters
        ----------
        x : float or array_like
            A angle or an array of angles. This array is flattened for
            the calculation. When an angle is provided, a float angle is returned.
        zero_2_2pi : bool, optional
            Change angle modulo range to [0, 2pi)
            Default is False.
        degree : bool, optional
            If True, then the given angles are assumed to be in degrees.
            Default is False.

        Returns
        -------
        ret : float or ndarray
            an angle or an array of modulated angle.

        Examples
        --------
        >>> angle_mod(-4.0)
        2.28318531

        >>> angle_mod([-4.0])
        np.array(2.28318531)

        >>> angle_mod([-150.0, 190.0, 350], degree=True)
        array([-150., -170.,  -10.])

        >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
        array([300.])

        """
        if isinstance(x, float):
            is_float = True
        else:
            is_float = False

        x = np.asarray(x).flatten()
        if degree:
            x = np.deg2rad(x)

        if zero_2_2pi:
            mod_angle = x % (2 * np.pi)
        else:
            mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

        if degree:
            mod_angle = np.rad2deg(mod_angle)

        if is_float:
            return mod_angle.item()
        else:
            return mod_angle

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        return self.angle_mod(angle)


    def calc_target_index(self, state, cx, cy):
        """
        Compute index in the trajectory list of the target.

        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + L * np.cos(state.yaw)
        fy = state.y + L * np.sin(state.yaw)

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)

        # Normalize trajectory indices
        trajectory_indices = np.arange(len(cx))
        normalized_indices = trajectory_indices / len(cx)

        # Introduce weighting for the order in the trajectory
        weight_order = 0.3  # weight for the order in the trajectory, adjust as necessary
        weighted_d = d + weight_order * normalized_indices

        target_idx = np.argmin(weighted_d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                        -np.sin(state.yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle


    def listener_state_callback(self, msg):
        # Update the state of the robot
        self.state = msg

    def listener_path_callback(self, msg):
        # Update the desired path
        if self.path == None or msg.id != self.path.id:
            self.path = msg
            if self.state != None:
                self.target_idx, _ = self.calc_target_index(self.state, self.path.cx, self.path.cy)
                self.last_idx = len(self.path.cx) - 1
    
    def listener_controller_state_callback(self, msg):
        #Update the planner state
        if msg.controller_state == "run" and self.ignition_flag == False:
            #self.ignition()
            self.ignition_flag = True
        self.controller_state = msg
    
    def check_goal(self):
        print(self.target_idx)
        print(self.last_idx)
        if self.last_idx == self.target_idx:
            self.current_controller_feedback = "goal_reached"
            return True
        return False
    
    def clamp(self, value):
        return max(-32, min(value, 32))
    
    def ignition(self):
        msg = AbraVehicleControl()
        speed = 0
        for i in range(10):
            msg.linear_speed = float(speed)
            self.publisher_.publish(msg)
            time.sleep(0.2)
            speed+=0.15
            print(speed)
        
            
    def publish_steering_angle(self):
        # Calculate the steering angle using the provided code
        # and publish it to the '/abra/cmd_vel' topic
        if self.ignition_flag == True:
            if self.state != None and self.path != None and self.target_idx != None:
                # Convert Pose and Path messages to your state and path representations
                # Calculate the steering angle
                delta, self.target_idx = self.stanley_control(self.state, self.path.cx, self.path.cy, self.path.cyaw, self.target_idx)
                # Create a Twist message
                control_signal = Twist()
                # control_signal = AbraVehicleControl()
                control_signal.linear.x = 0.0
                # Set the steering angle
                if not self.check_goal() and self.controller_state.controller_state == "run":
                    control_signal.linear.x = float(1.34)
                    #control_signal.steering_angle = self.clamp(int(-math.degrees(delta)))
                    control_signal.angular.z = float(delta)
                    self.current_controller_feedback = "on_the_way"
                else:
                    control_signal.linear.x = float(0.0)
                    control_signal.angular.z = float(0.0)
                # Publish the message
                self.publisher_.publish(control_signal)
                feedback = String()
                feedback.data = self.current_controller_feedback
                self.feedback_publisher.publish(feedback)
                print("published")
                print(control_signal)

def main(args=None):
    rclpy.init(args=args)
    path_tracking_node = PathTrackingNode()
    rclpy.spin(path_tracking_node)
    path_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
