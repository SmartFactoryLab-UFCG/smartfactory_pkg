#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from turtlebot2i_interfaces.srv import ListMaps

class ListMapsServer(Node):

    def __init__(self):
        super().__init__('list_maps_server')
        self.srv = self.create_service(ListMaps, 'list_maps', self.list_maps_callback)

    def list_maps_callback(self, request, response):
        maps_dir = '/workspaces/robotxr/turtlebot2i_ws/maps'
        map_names = [f for f in os.listdir(maps_dir) if os.path.isfile(os.path.join(maps_dir, f))]
        response.map_names = map_names
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ListMapsServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
