# Tests require running mineros and minecraft
import copy
import math
import threading
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from javascript import AsyncTask, On, Once, off, once, require
from rclpy.node import Node

from mineros_inter.msg import BlockPose, Item
from mineros_inter.srv import (BlockInfo, Craft, FindBlocks, Inventory,
                                    MineBlock, PlaceBlock)


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

        self.find_y_client = self.create_client(
            BlockInfo,
            '/mineros/findy'
        )
        
        self.craft_client = self.create_client(
            Craft,
            '/mineros/interaction/craft'
        )

        self.tests()

    def position_cb(self, msg: PoseStamped):
        self.position = msg

    def tests(self):
        # for i in range(5):
        #     self.test_mine_block(blockid=179, count=1)
        #     self.test_mine_block(blockid=41, count=1)

        # self.test_find_grass()
        # self.test_inventory()
        # self.test_place_block() 
        self.test_craft_crafting_table()
        self.test_craft_wood_axe()
        self.test_craft_iron_pick()

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

    def test_mine_block(self, blockid=46, count=2):

        blocks = self.find_blocks(blockid, count)

        self.get_logger().info('Mine block test started')
        self.get_logger().info(f'Found {len(blocks)} blocks')

        for i in range(len(blocks)):
            block = blocks[i]
            assert self.mine_block(block)

        self.get_logger().info('Mine block test passed')

    def find_blocks(self, blockid: int, count: int) -> List[Pose]:
        self.get_logger().info('Find blocks')
        block_search = FindBlocks.Request()
        block_search.blockid = blockid
        block_search.count = count
        block_search.max_distance = 30

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

    def test_place_block(self, blockid=15):
        while self.position is None:
            self.get_logger().info('Waiting for position')
            rclpy.spin_once(self)

        request = PlaceBlock.Request()
        point = copy.deepcopy(self.position.pose.position)
        point.x += 2.0
        point = self.find_y(point).position
        item = Item()
        item.id = blockid
        item.count = 1

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
        while not self.place_blocks_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service')
        future = self.place_blocks_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        assert future.result().success

        self.get_logger().info('Place block test passed')
    

    def find_y(self, position: Point) -> Pose:
        req = BlockInfo.Request()
        req.block_pose = Pose()
        req.block_pose.position = position

        future = self.find_y_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        block = future.result()
        return block.block.block_pose

    def test_craft_crafting_table(self):
        self.get_logger().info('Crafting test started')
        blocks = self.find_blocks(49, 4) # 49 is jungle log 
        self.get_logger().info(f'Found {len(blocks)} blocks')
        for block in blocks:
            self.mine_block(block)
        
        crafting_req = Craft.Request()
        item = Item()
        item.id = 26 # jungle planks
        item.count = 4
        
        crafting_req.item = item
        crafting_req.crafting_table = False
        
        future = self.craft_client.call_async(crafting_req)
        rclpy.spin_until_future_complete(self, future)
        assert future.result().success
        
            
        crafting_req = Craft.Request()
        item = Item()
        item.id = 278
        item.count = 1
        
        crafting_req.item = item
        crafting_req.crafting_table = False
        
        while not self.craft_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service')
        future = self.craft_client.call_async(crafting_req)
        rclpy.spin_until_future_complete(self, future)
        assert future.result().success
        
        self.test_place_block(278)
        
    def test_craft_wood_axe(self):
        # Craft planks
        crafting_req = Craft.Request()
        item = Item()
        item.id = 26
        item.count = 8
        crafting_req.item = item
        crafting_req.crafting_table = False
        
        future = self.craft_client.call_async(crafting_req)
        rclpy.spin_until_future_complete(self, future)
        
        # Craft sticks
        crafting_req = Craft.Request()
        item = Item()
        item.id = 807
        item.count = 4
        crafting_req.item = item
        crafting_req.crafting_table = False
        
        future = self.craft_client.call_async(crafting_req)
        rclpy.spin_until_future_complete(self, future)
        
        # Craft axe
        crafting_req = Craft.Request()
        item = Item()
        item.id = 780
        item.count = 1
        crafting_req.item = item
        crafting_req.crafting_table = True
        
        # Find crafting table
        blocks = self.find_blocks(182, 1)
        assert len(blocks) > 0
        block: Pose = blocks[0]
        crafting_req.crafting_table_location = block
        
        future = self.craft_client.call_async(crafting_req)
        rclpy.spin_until_future_complete(self, future)
        assert future.result().success

    def test_craft_iron_pick(self):
        
        # Craft axe
        crafting_req = Craft.Request()
        item = Item()
        item.id = 794
        item.count = 1
        crafting_req.item = item
        crafting_req.crafting_table = True
        crafting_req.danger_mode = True
        
        # Find crafting table
        blocks = self.find_blocks(182, 1)
        assert len(blocks) > 0
        block: Pose = blocks[0]
        crafting_req.crafting_table_location = block
        
        
        future = self.craft_client.call_async(crafting_req)
        rclpy.spin_until_future_complete(self, future)
        assert future.result().success
        


def main(args=None):
    rclpy.init(args=args)
    node = MiningTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
