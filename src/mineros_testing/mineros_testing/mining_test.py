# Tests require running mineros and minecraft
import math
import threading
import copy
import rclpy
from rclpy.node import Node
from javascript import require, On, Once, AsyncTask, once, off
from typing import List, Tuple

from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached, State

from geometry_msgs.msg import PoseStamped, Point
from mineros_interfaces.srv import FindBlocks, MineBlocks, Inventory, PlaceBlocks
from mineros_interfaces.msg import Item, BlockPose


class MiningTestNode(Node):
    def __init__(self):
        super().__init__('mining_tester')

        self.position: PoseStamped = None

        self.local_position_sub = self.create_subscription(
            PoseStamped,
            '/mineros/local_position/pose',
            self.position_cb,
            10
        )

        self.find_block_client = self.create_client(
            FindBlocks,
            '/mineros/mining/find_blocks',
        )

        self.mine_block_client = self.create_client(
            MineBlocks,
            '/mineros/mining/mine_blocks'
        )

        self.inventory_contents_service = self.create_client(
            Inventory,
            '/mineros/inventory/contents'
        )

        self.place_blocks_client = self.create_client(
            PlaceBlocks,
            '/mineros/interaction/place_blocks',
        )

        self.tests()

    def position_cb(self, msg: PoseStamped):
        self.position = msg

    def tests(self):
        self.test_find_grass()
        # self.test_mine_block()
        item = self.test_inventory()
        self.test_place_block(item)

        self.destroy_node()
        rclpy.shutdown()

    def test_find_grass(self):
        block_search = FindBlocks.Request()
        block_search.blockid = 8

        self.get_logger().info('Find grass test started')
        while not self.find_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Find grass test waiting for service')

        self.get_logger().info('Find grass test started')

        future = self.find_block_client.call_async(block_search)
        rclpy.spin_until_future_complete(self, future)
        blocks = future.result()

        assert len(blocks.blocks.poses) > 0
        self.get_logger().info('Find grass test passed')
        self.get_logger().info(
            f'Found {len(blocks.blocks.poses)} grass blocks')

    def test_mine_block(self):
        block_search = FindBlocks.Request()
        block_search.blockid = 8
        block_search.count = 3
        while not self.find_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service')
        future = self.find_block_client.call_async(block_search)
        rclpy.spin_until_future_complete(self, future)
        blocks = future.result()

        req = MineBlocks.Request()
        req.blocks = blocks.blocks

        self.get_logger().info('Mine block test started')
        self.get_logger().info(f'Mining {len(blocks.blocks.poses)} blocks')
        future = self.mine_block_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        success = future.result()
        assert success
        self.get_logger().info('Mine block test passed')

    def test_inventory(self):
        req = Inventory.Request()
        fut = self.inventory_contents_service.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

        inventory = fut.result()
        assert len(inventory.inventory) > 0
        self.get_logger().info('Inventory test passed')
        return inventory.inventory[0]

    def test_place_block(self, item: Item):
        while self.position is None:
            self.get_logger().info('Waiting for position')
            rclpy.spin_once(self)

        request = PlaceBlocks.Request()
        blocks = []
        point = copy.deepcopy(self.position.pose.position)
        point.x -= 10
        point.y -= 1

        point.x = float(math.ceil(point.x))
        point.z = float(math.ceil(point.z))

        for i in range(3):
            point.x -= i * 2
            blockpose = BlockPose()
            blockpose.block = item
            blockpose.point = copy.deepcopy(point)
            blockpose.face_vector = Point()
            blockpose.face_vector.x = 0.0
            blockpose.face_vector.y = 1.0
            blockpose.face_vector.z = 0.0
            self.get_logger().info(f'Placing block at {point}')
            self.get_logger().info(f'{blockpose}')
            blocks.append(blockpose)

        self.get_logger().info(f'{blocks}')
        request.blocks = blocks

        fut = self.place_blocks_client.call_async(request)
        rclpy.spin_until_future_complete(self, fut)
        success = fut.result()

        assert all(success.success)
        self.get_logger().info('Place block test passed')


def main(args=None):
    rclpy.init(args=args)
    node = MiningTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
