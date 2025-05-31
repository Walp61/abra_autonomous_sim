#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

class TrafficLightSpawner(Node):
    def __init__(self):
        super().__init__('traffic_light_spawner')

        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=0.01):
            self.get_logger().info('spawn_entity service waiting...')
        
        self.cli_delete = self.create_client(DeleteEntity, '/delete_entity')
        while not self.cli_delete.wait_for_service(timeout_sec=0.01):
            self.get_logger().info('delete_entity ervice waiting...')
#---------------------------------------------------------------------
        pkg_path = get_package_share_directory('abra_gazebo')

        red_sdf_file_path = os.path.join(pkg_path, 'models', 'KIRMIZI', 'model.sdf')
        red1_sdf_file_path2 = os.path.join(pkg_path, 'models', 'KIRMIZI2', 'model.sdf')

        green_sdf_file_path = os.path.join(pkg_path, 'models', 'YESIL2', 'model.sdf')
        green1_sdf_file_path2 = os.path.join(pkg_path, 'models', 'YESIL', 'model.sdf')

        with open(red_sdf_file_path, 'r') as f:
            self.red_sdf_content = f.read()
        
        with open(red1_sdf_file_path2 , 'r') as f:
            self.red1_sdf_content = f.read()

        with open(green_sdf_file_path, 'r') as f:
            self.green_sdf_content = f.read()
        
        with open(green1_sdf_file_path2 , 'r') as f:
            self.green1_sdf_content = f.read()

        self.current_light = 'red'
        self.spawn_red()
        self.timer = self.create_timer(10.0, self.change_light)

    def spawn_red(self):

        req = SpawnEntity.Request()
        req.name = 'traffic_light_1'
        req.xml = self.red_sdf_content
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose.position.x = 28.5
        req.initial_pose.position.y = -1.7
        req.initial_pose.position.z = 0.0

        req2 = SpawnEntity.Request()
        req2.name = 'traffic_light_2'
        req2.xml = self.red1_sdf_content
        req2.robot_namespace = ''
        req2.reference_frame = 'world'
        req2.initial_pose.position.x = 27.0
        req2.initial_pose.position.y = 52.4
        req2.initial_pose.position.z = 0.0

        self.cli.call_async(req)
        self.cli.call_async(req2)

    def spawn_green(self):

        req = SpawnEntity.Request()
        req.name = 'traffic_light_3'
        req.xml = self.green_sdf_content
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose.position.x = 28.5
        req.initial_pose.position.y = -1.7
        req.initial_pose.position.z = 0.0

        req2 = SpawnEntity.Request()
        req2.name = 'traffic_light_4'
        req2.xml = self.green1_sdf_content
        req2.robot_namespace = ''
        req2.reference_frame = 'world'
        req2.initial_pose.position.x = 27.0
        req2.initial_pose.position.y = 52.4
        req2.initial_pose.position.z = 0.0

        self.cli.call_async(req)
        self.cli.call_async(req2)

    def delete_red(self):
        req = DeleteEntity.Request()
        req2 = DeleteEntity.Request()
        req.name = 'traffic_light_1'
        req2.name = 'traffic_light_2'
        self.cli_delete.call_async(req)
        self.cli_delete.call_async(req2)

    def delete_green(self):
        req = DeleteEntity.Request()
        req2 = DeleteEntity.Request()
        req.name = 'traffic_light_3'
        req2.name = 'traffic_light_4'
        self.cli_delete.call_async(req)
        self.cli_delete.call_async(req2)

    def change_light(self):
        
        if self.current_light=='red':
            self.delete_red()
            self.current_light='green'
            self.spawn_green()
        else:
            self.delete_green()
            self.current_light='red'
            self.spawn_red()

    
def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
