# Tests require running mineros and minecraft
import math
import threading
import copy
import rclpy
from rclpy.node import Node
from javascript import require, On, Once, AsyncTask, once, off
from typing import List, Tuple


from geometry_msgs.msg import PoseStamped, Point, Pose
from mineros_interfaces.srv import FindBlocks, MineBlock, Inventory, PlaceBlock, Craft
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
            MineBlock,
            '/mineros/mining/mine_block'
        )

        self.inventory_contents_service = self.create_client(
            Inventory,
            '/mineros/inventory/contents'
        )

        self.place_blocks_client = self.create_client(
            PlaceBlock,
            '/mineros/interaction/place_block',
        )

        self.tests()

    def position_cb(self, msg: PoseStamped):
        self.position = msg

    def tests(self):
        self.test_find_grass()
        self.test_mine_block()
        item = self.test_inventory()
        # self.test_place_block(item) # WARNING: This test is broken
        self.test_craft_crafting_table()

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

    def test_mine_block(self, blockid=8, count=1):
        
        blocks = self.find_blocks(blockid, count)

        self.get_logger().info('Mine block test started')
        self.get_logger().info(f'Found {len(blocks)} blocks')

        for i in range(len(blocks)):
            block = blocks[i]
            assert self.mine_block(block)

        self.get_logger().info('Mine block test passed')

    def find_blocks(self, blockid: int, count: int) -> List[Pose]:
        block_search = FindBlocks.Request()
        block_search.blockid = blockid
        block_search.count = count
        
        while not self.find_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service')
        future = self.find_block_client.call_async(block_search)
        rclpy.spin_until_future_complete(self, future)
        blocks = future.result()
        return blocks.blocks.poses
    
    def mine_block(self, block: Pose) -> bool:
        req = MineBlock.Request()
        req.block = block

        future = self.mine_block_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        success = future.result()

        return success.success

    def test_inventory(self):
        req = Inventory.Request()
        fut = self.inventory_contents_service.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

        inventory = fut.result()
        assert len(inventory.inventory) > 0
        self.get_logger().info('Inventory test passed')
        return inventory.inventory[0]


    # TODO: Fix this test
    def test_place_block(self, item: Item):
        while self.position is None:
            self.get_logger().info('Waiting for position')
            rclpy.spin_once(self)

        request = PlaceBlock.Request()
        point = copy.deepcopy(self.position.pose.position)
        point.x -= 10
        point.y -= 1

        point.x = float(math.ceil(point.x))
        point.z = float(math.ceil(point.z))

        blockpose = BlockPose()
        blockpose.block = item
        blockpose.block_pose = Pose()
        blockpose.block_pose.position = point
        blockpose.face_vector = Point()
        blockpose.face_vector.x = 0.0
        blockpose.face_vector.y = 1.0
        blockpose.face_vector.z = 0.0
        self.get_logger().info(f'Placing block at {point}')
        self.get_logger().info(f'{blockpose}')

        request.block = blockpose

        fut = self.place_blocks_client.call_async(request)
        rclpy.spin_until_future_complete(self, fut)
        success = fut.result()

        assert success.success
        self.get_logger().info('Place block test passed')

    # TODO: figure out the id for the wood block
    def test_craft_crafting_table(self):
        self.get_logger().info('Crafting test started')
        blocks = self.find_blocks(47,4)
        self.get_logger().info(f'Found {len(blocks)} blocks')
        for block in blocks:
            self.mine_block(block)
        crafting_req = Craft.Request()
        
        


def main(args=None):
    rclpy.init(args=args)
    node = MiningTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
