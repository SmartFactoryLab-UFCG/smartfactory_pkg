#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rtabmap_msgs.srv import LoadDatabase

class LoadDatabaseClient(Node):

    def __init__(self):
        super().__init__('load_database_client')
        self.cli = self.create_client(LoadDatabase, 'load_database')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LoadDatabase.Request()

    def send_request(self, database_path, clear):
        if not os.path.exists(database_path):
            self.get_logger().info(f'Creating new database at {database_path}')
        self.req.database_path = database_path
        self.req.clear = clear
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        response = self.future.result()
        if response is None:
            self.get_logger().error('No response received. Service might be unavailable.')
            return None
        return response

def main(args=None):
    rclpy.init(args=args)

    client = LoadDatabaseClient()
    database_path = '/workspaces/robotxr/turtlebot2i_ws/maps/map2.db'
    response = client.send_request(database_path, False)

    if response:
        client.get_logger().info(f'Database loaded successfully. Response: {repr(response)}')
    else:
        client.get_logger().error(f'Failed to load database at {database_path}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

