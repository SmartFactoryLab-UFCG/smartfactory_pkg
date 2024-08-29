#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlebot2i_interfaces.srv import ListMaps

class ListMapsClient(Node):

    def __init__(self):
        super().__init__('list_maps_client')
        self.cli = self.create_client(ListMaps, 'list_maps')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ListMaps.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = ListMapsClient()
    response = client.send_request()
    client.get_logger().info(f'Response: {response.map_names}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
