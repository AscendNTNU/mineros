# Tests require running mineros and minecraft
import math
import threading
import time
import rclpy
from rclpy.node import Node
from javascript import require, On, Once, AsyncTask, once, off
from typing import List, Tuple

from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached, State

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from mineros_interfaces.srv import FindBlocks


class MiningTestNode(Node):
    def __init__(self):
        super().__init__('mining_tester')
        self.position: PoseStamped = None

        self.find_block_client = self.create_client(
            FindBlocks,
            '/mineros/mining/find_blocks',
        )
      
        self.tests()
        # threading.Thread(target=self.tests).start()

    def tests(self):
        self.test_find_grass()
        
        self.destroy_node()
        rclpy.shutdown()

    def test_find_grass(self):
        block_search = FindBlocks.Request()
        block_search.blockid = 8
        block_search.count = 10
        
        self.get_logger().info('Find grass test started')
        while not self.find_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Find grass test waiting for service')
        
        self.get_logger().info('Find grass test started')
        
        
        future = self.find_block_client.call_async(block_search)
        rclpy.spin_until_future_complete(self, future)
        blocks = future.result()
        
        assert len(blocks.blocks.poses) > 0
        self.get_logger().info('Find grass test passed')
        self.get_logger().info(f'Found {len(blocks.blocks.poses)} grass blocks')
   
def main(args=None):
    rclpy.init(args=args)
    node = MiningTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
