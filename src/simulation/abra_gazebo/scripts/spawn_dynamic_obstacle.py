#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
import math, os

class MovingObjectSpawner(Node):
    def __init__(self):
        super().__init__('moving_object_spawner')

        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=0.01):
            self.get_logger().info('spawn_entity service waiting...')
        
        pkg_path = get_package_share_directory('abra_gazebo')
        engel_sdf_file_path = os.path.join(pkg_path, 'models', 'ENGEL', 'model.sdf')
        with open(engel_sdf_file_path, 'r') as f:
            self.engel_sdf_content = f.read()
        
        self.y = 4.0

        self.spawn_model()
        self.pub = self.create_publisher(Twist, '/obstacle/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.move_callback)

        
    def spawn_model(self):
        
        req = SpawnEntity.Request()
        req.name = 'obstacle'
        req.xml = self.engel_sdf_content
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose.position.x = 16.0  # X pozisyonu sabit
        req.initial_pose.position.y = self.y  # Y pozisyonu sabit
        req.initial_pose.position.z = 0.0   # Z pozisyonu sabit

        self.cli.call_async(req)

    def move_callback(self):  
        msg = Twist()
        msg.linear.y = -0.5
        msg.angular.z = 0.0
        if self.y<=-2.4:
            msg.linear.y = 0.0
            self.pub.publish(msg)
            raise Exception("Model sinira ulasti!")
        self.pub.publish(msg)
        self.y -=0.5

        


def main(args=None):
    rclpy.init(args=args)
    node = MovingObjectSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
