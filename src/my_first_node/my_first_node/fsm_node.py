import rclpy
import json
import os
import copy

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from typing import Any
from geometry_msgs.msg import PoseStamped, Pose, Point

from mineros_inter.msg import BlockPose, Item
from mineros_inter.srv import (BlockInfo, BotPos, MoveTo, Craft, Inventory, Recipe, FindBlocks, MineBlock, PlaceBlock)

class MyFirstFSM(Node):
    """
    This class encompases the skeleton of a ROS2 node. It inherits from the class Node, which is the base class
    for all nodes in ROS2. The class Node is defined in the rclpy package. The class Node has a constructor that
    takes the name of the node as an argument. The name of the node is the name that will be used to identify the
    node in the ROS2 ecosystem. The name of the node must be unique in the ROS2 ecosystem. If two nodes have the
    same name, then the second node will not be able to start.
    
    The aim of this node will be to create a small node that will allow the bot to move around and navigate its
    space
    """
    def __init__(self) -> None:
        """
        The constructor of the class. It initializes the node with the name 'my_first_node'. In this method
        we also initialize the publishers and subscribers that we want to use in the node. As well as all the possible
        parameters that we want to be able to change from the launch file.
        """
        super().__init__('my_first_node')
        #self.position: PoseStamped = None
        self.blockIDDic = {}
        self.itemIDDic  = {}
        self.recipeIDDic = {}
        closestCraftingTable = Point()

        # This is how you print to terminal in a ros node
        self.get_logger().info('FSM running')
        

        # Create a publisher for setting the position
        #self.position_publisher = self.create_publisher(
        #    PoseStamped,
        #    '/mineros/local_position/pose',
        #    10
        #)

        # Create a client for getting the bot position
        self.bot_pos_client = self.create_client(
            BotPos,
            '/mineros/get_bot_position',
        )

        self.set_position_client = self.create_client(
            MoveTo,
            '/mineros/set_position',
        )
        
        self.look_at_block_client = self.create_client(
            MoveTo,
            '/mineros/look_at_block',
        )

        self.block_info_client = self.create_client(
            BlockInfo,
            '/mineros/blockinfo'
        )

        self.find_block_client = self.create_client(
            FindBlocks,
            '/mineros/mining/find_blocks',
        )

        self.mine_block_client = self.create_client(
            MineBlock,
            '/mineros/mining/mine_block',
        )
        
        self.craft_client = self.create_client(
            Craft,
            '/mineros/interaction/craft'
        )

        self.get_inventory_client = self.create_client(
            Inventory,
            '/mineros/inventory/content'
        )

        self.get_recipe_client = self.create_client(
            Recipe,
            '/mineros/interaction/recipe'
        )

        self.place_block_client = self.create_client(
            PlaceBlock,
            '/mineros/interaction/place_block',
        )

        # Import json data
        fileNames = ['blocks', 'items', 'recipes']
        dictionaries = []
        shareDir = get_package_share_directory('my_first_node')
        filePaths = [os.path.join(shareDir, f'json_files/{fileName}.json') for fileName in fileNames]

        # Read the JSON file and convert to dictionary
        with open(filePaths[0], 'r') as file:
            jsonFile = json.load(file) 
            for item in jsonFile:
                self.blockIDDic[item['name']] = item['id']

        with open(filePaths[1], 'r') as file:
            jsonFile = json.load(file) 
            for item in jsonFile:
                self.itemIDDic[item['name']] = item['id']

        with open(filePaths[2], 'r') as file:
            jsonFile = json.load(file) 
            for item in jsonFile:
                block_name = ''
                for name, ID in self.itemIDDic.items():
                    if ID == int(item):
                        block_name = name
                self.recipeIDDic[block_name] = jsonFile[item]

        self.getting_started()

    # ------------------------------
    # Service calls
    # ------------------------------
    
    def move_to(self, move_to_position):
        request = MoveTo.Request()
        request.pose = move_to_position

        while not self.set_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/set_position')
        self.set_position_client.wait_for_service()
        future = self.set_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def get_position(self):
        request = BotPos.Request()

        while not self.bot_pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/get_bot_position')
        future = self.bot_pos_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().bot_pos
    
    def craft_item(self, itemName, count, useCraftingTable=False):
        request = Craft.Request()
        request.item.id = self.itemIDDic.get(itemName)
        request.item.count = count

        if useCraftingTable:
            if self.closestCraftingTable is None:
                # TODO: Find closest crafting table if possible, or else craft one etc.
                self.get_logger().info('No crafting table found')
                return False
            
            tableBlockID = self.get_block_info(self.closestCraftingTable).block.block.id

            if not tableBlockID == self.blockIDDic.get('crafting_table'):
                self.get_logger().info(f'No crafting table at: {self.closestCraftingTable.x}, {self.closestCraftingTable.y}, {self.closestCraftingTable.z}')
                return False
            
            self.get_logger().info(f'Using crafting table at: {self.closestCraftingTable.x}, {self.closestCraftingTable.y}, {self.closestCraftingTable.z}')

            request.crafting_table = True
            request.crafting_table_location.position = self.closestCraftingTable
            
        while not self.craft_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/interaction/craft')
        future = self.craft_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        #TODO: Gets stuck when no items are available
        self.get_logger().info(f'Crafted {count} {itemName}')
        #TODO: If missing items, return reason and missing items
        return future.result().success


    def get_inventory(self):
        request = Inventory.Request()

        while not self.get_inventory_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/inventory/content')
        future = self.get_inventory_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_recipe(self, itemID, count):
        request = Recipe.Request()
        request.item.id = itemID
        request.item.count = count

        while not self.get_recipe_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/interaction/recipe')
        future = self.get_recipe_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().recipies

    def get_block_info(self, blockPosition):
        request = BlockInfo.Request()
        request.block_pose.position = blockPosition

        while not self.block_info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/blockinfo')
        future = self.block_info_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def find_blocks(self, blockName, count, distance):
        request = FindBlocks.Request()
        blockID = self.blockIDDic.get(blockName)
        request.blockid = blockID
        request.max_distance = distance
        request.count = count

        while not self.find_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/mining/find_blocks')
        future = self.find_block_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().blocks.poses

    def mine_block(self, blockPosition):
        req = MineBlock.Request()
        req.block.position = blockPosition
    
        while not self.mine_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/mining/mine_block')
        future = self.mine_block_client.call_async(req)
        self.get_logger().info(f'Mining block at: ({blockPosition.x}, {blockPosition.y}, {blockPosition.z})')
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
    
    def place_block(self, pointToPlace: Point, itemName, faceVector: Point):
        req = PlaceBlock.Request()
        blockPose = BlockPose()
        blockPose.block_pose = Pose()
        blockPose.block_pose.position = pointToPlace
        blockPose.face_vector = faceVector
        blockPose.block.id = self.itemIDDic.get(itemName)
        blockPose.block.count = 1
        req.block = blockPose
    
        while not self.place_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /mineros/mining/place_block')
        future = self.place_block_client.call_async(req)
        self.get_logger().info(f'Placing {itemName} at: ({pointToPlace.x}, {pointToPlace.y}, {pointToPlace.z})')
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    # ------------------------------
    # Helper functions
    # ------------------------------

    def search_and_mine(self, blockName, count):
        blocks = self.find_blocks(blockName, count, 30)
        failed_mine_count = 0
        original_count = 0

        #inventory = self.get_inventory()
        #for item in inventory.inventory:
        #    if item.id == blockID:
        #        count -= item.count

        for block in blocks:
            #TODO: Check if tool is required, and if so, get the tool
            self.mine_block(block.position)
            #if not self.mine_block(block.position):
            #    #TODO: Mining sometimes fails, check why
            #    failed_mine_count += 1

        self.get_logger().info(f'Failed to mine: {failed_mine_count} blocks')
        blocks_mined = len(blocks) - failed_mine_count
        self.get_logger().info(f'Mined: {blocks_mined} blocks')


        if blocks_mined < count:
            self.get_logger().info('Not enough blocks mined, searching for more')
            current_position = self.get_position()
            new_position = current_position
            new_position.pose.position.x += 40
            self.move_to(new_position)
            self.search_and_mine(blockName, count - blocks_mined)
        
    # ------------------------------
    # FSM functions
    # ------------------------------

    # Code for the basics of minecraft
    def getting_started(self):
        self.get_logger().info('Getting started')       

        self.search_and_mine('oak_log', 5)
        self.craft_item('oak_planks', 4)
        if not self.craft_item('crafting_table', 1):
            self.get_logger().info('Failed to craft crafting table')
            return
        # Place crafting table
        pointToPlace = self.get_position().pose.position
        pointToPlace.x += 1
        self.closestCraftingTable = copy.deepcopy(pointToPlace)
        self.get_logger().info(f'y crafting table: {self.closestCraftingTable.y}')
        pointToPlace.y -= 1     
        faceVector = Point(x=0.0, y=1.0, z=0.0)
        if not self.place_block(pointToPlace, 'crafting_table', faceVector):
            self.closestCraftingTable = None

        # Craft sticks
        self.craft_item('stick', 1)
        # craft pickaxe
        if not self.craft_item('wooden_pickaxe', 1, True):
            self.get_logger().info('Failed to craft pickaxe')
        

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    