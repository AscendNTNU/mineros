import math
import threading
import time
from threading import Thread
from javascript import require, On, Once, AsyncTask, once, off
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from mineros_interfaces.srv import FindBlocks, MineBlock, Inventory, PlaceBlock, Craft, BlockInfo
from mineros_interfaces.msg import Item, BlockPose

mineflayer = require('mineflayer')
pathfinder = require('mineflayer-pathfinder')
toolPlugin = require('mineflayer-tool').plugin
collectBlock = require('mineflayer-collectblock').plugin
Vec3 = require('vec3').Vec3

bot = mineflayer.createBot(
    {'host': 'localhost', 'port': 25565, 'username': 'MinerosBot', 'hideErrors': False})
bot.loadPlugin(pathfinder.pathfinder)
bot.loadPlugin(toolPlugin)
bot.loadPlugin(collectBlock)

# The spawn event
once(bot, 'login')
bot.chat('I spawned')

# Instantiating pathfinding
movements = pathfinder.Movements(bot)
bot.pathfinder.setMovements(movements)

# Events
goal_reached = False


@On(bot, 'goal_reached')
def on_goal_reached(_, goal):
    print(f'Goal reached')
    global goal_reached
    goal_reached = True
    bot.chat('I reached the goal')


digging_completed = False


@On(bot, 'diggingCompleted')
def on_digging_completed(_, block):
    global digging_completed
    print('Digging completed')
    digging_completed = True
    bot.chat('I finished digging')


class MinerosMain(Node):
    def __init__(self):
        super().__init__('mineros_main_node')
        self.get_logger().info("Hello World!")

        self.declare_parameter('goal_acceptance', 1.0)
        self.declare_parameter('goal_timeout', 30)
        self.declare_parameter('digging_timeout', 10)

        self.goal_acceptance = self.get_parameter('goal_acceptance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.digging_timeout = self.get_parameter('digging_timeout').value

        self.mission = []
        self.mission_thread = None
        self.current_position = bot.entity.position

        timers_cbg = ReentrantCallbackGroup()

        # Movement control
        self.find_y_service = self.create_service(
            BlockInfo,
            '/mineros/findy',
            self.find_y_callback
        )

        self.set_potision = self.create_subscription(
            PoseStamped,
            '/mineros/set_position',
            self.set_position_callback,
            10
        )

        self.set_position_composite = self.create_subscription(
            PoseArray,
            '/mineros/set_position/composite',
            self.set_position_composite_callback,
            10
        )

        self.position_reached_publisher = self.create_publisher(
            Empty,
            '/mineros/set_position/reached',
            10
        )

        # Mining control
        self.find_blocks_service = self.create_service(
            FindBlocks,
            '/mineros/mining/find_blocks',
            self.find_blocks_callback
        )

        self.mine_block_service = self.create_service(
            MineBlock,
            '/mineros/mining/mine_block',
            self.mine_block_callback
        )

        self.inventory_contents_service = self.create_service(
            Inventory,
            '/mineros/inventory/contents',
            self.inventory_contents_service_callback
        )

        # Interaction control
        self.place_block_service = self.create_service(
            PlaceBlock,
            '/mineros/interaction/place_block',
            self.place_block_callback,
        )

        self.craft_item_service = self.create_service(
            Craft,
            '/mineros/interaction/craft',
            self.craft_item_service_callback
        )

        # Info publishers
        self.local_position_publisher = self.create_publisher(
            PoseStamped,
            '/mineros/local_position/pose',
            10
        )

        self.pose_timer = self.create_timer(
            0.1,
            self.local_pose_timer_callback,
            callback_group=timers_cbg
        )

        # Write the items to text file for documentation
        mc_data = require('minecraft-data')(bot.version)
        items = mc_data.items
        with open('docs/items.txt', 'w') as outfile:
            outfile.write(str(items))

    def spin_for_goal(self):
        global goal_reached
        start = time.time()
        while not goal_reached:
            time.sleep(0.1)
            if time.time() - start > self.goal_timeout:
                self.get_logger().info("Goal timeout")
                return
        goal_reached = False

    def spin_for_digging(self):
        global digging_completed
        start = time.time()

        while not digging_completed:
            time.sleep(0.1)
            if time.time() - start > self.digging_timeout:
                self.get_logger().info("Digging timeout")
                return
        digging_completed = False

    def calculate_wait_time(self, target: Vec3):
        """Calculates the time it takes to get to a target position on average"""
        current_position = bot.entity.position
        distance = math.sqrt((target.x - current_position.x)**2 + (target.y -
                             current_position.y)**2 + (target.z - current_position.z)**2)

        self.get_logger().info(f"time: {distance/5}")
        return distance / 5  # Steves average speed is 5.6 blocks per second, 5 is a safe bet

    def find_y_callback(self, request: BlockInfo, response: BlockInfo):
        """Returns the ground level y coordinate for a given x and z coordinate"""
        self.get_logger().info("find_y_callback")

        response.block = BlockPose()
        response.block.block_pose = Pose()
        response.block.block_pose.position.x = request.block_pose.position.x
        response.block.block_pose.position.z = request.block_pose.position.z
        self.get_logger().info(
            f'find_y_callback: {request.block_pose.position.x}, {request.block_pose.position.z}')

        bot_position = bot.entity.position
        block_at_bot_height = bot.blockAt(Vec3(
            request.block_pose.position.x, bot_position.y, request.block_pose.position.z))

        iteration = 1
        if block_at_bot_height.type == 0:
            iteration = -1

        block = block_at_bot_height
        while True:
            self.get_logger().info(
                f'{block.type} , {block.name}, {block.position.y}, {iteration}')
            previous_block = block
            new_pose = Vec3(request.block_pose.position.x,
                            previous_block.position.y + iteration, request.block_pose.position.z)
            block = bot.blockAt(new_pose)

            # Iterated from dirt to sky
            if block.type == 0 and previous_block.type != 0:
                self.get_logger().info(
                    f'Found y: {block.type}, {block.position.y}, {iteration}')
                response.block.block_pose.position.y = float(
                    previous_block.position.y)
                return response
            # from sky to dirt
            elif block.type != 0 and previous_block.type == 0:
                self.get_logger().info(
                    f'Found y: {previous_block.type}, {previous_block.position.y}, {iteration}')
                response.block.block_pose.position.y = float(block.position.y)
                return response

    def set_position_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Set position")
        bot.pathfinder.setGoal(None)

        if msg.pose.position.y == -1.0:
            goal = pathfinder.goals.GoalNearXZ(
                round(msg.pose.position.x, 2), round(msg.pose.position.z, 2), self.goal_acceptance)
        else:
            goal = pathfinder.goals.GoalNear(
                round(msg.pose.position.x, 2), round(msg.pose.position.y, 2), round(msg.pose.position.z, 2), self.goal_acceptance)

        bot.pathfinder.setGoal(goal)

        # Wait for goal to be reached
        self.spin_for_goal()
        self.position_reached_publisher.publish(Empty())

    def set_position_composite_callback(self, msg: PoseArray):
        self.get_logger().info(f"Set position composite")

        bot.pathfinder.setGoal(None)
        for pose in msg.poses:
            ps = PoseStamped()
            ps.pose = pose
            self.set_position_callback(ps)

    def find_blocks_callback(self, request: FindBlocks.Request, response: FindBlocks.Response):
        self.get_logger().info(f"Find blocks: {request.blockid}")

        options = {}
        options['matching'] = request.blockid
        if request.max_distance != 0:
            options['maxDistance'] = request.max_distance
        if request.count != 0:
            options['count'] = request.count

        blocks = bot.findBlocks(options)

        assert blocks is not None

        pa = PoseArray()
        for block in blocks:
            p = Pose()
            p.position.x = float(block.x)
            p.position.y = float(block.y)
            p.position.z = float(block.z)
            pa.poses.append(p)

        response.blocks = pa
        return response

    def mine_block_callback(self, request: MineBlock.Request, response: MineBlock.Response):
        pose: Pose = request.block

        vec = Vec3(pose.position.x, pose.position.y, pose.position.z)
        block = bot.blockAt(vec)

        # Get to block
        bot.pathfinder.setGoal(None)
        goal = pathfinder.goals.GoalLookAtBlock(block.position, bot.world)
        bot.pathfinder.setGoal(goal)
        self.spin_for_goal()

        try:
            bot.collectBlock.collect(block, timeout=20)
            global block_to_monitor
            block_to_monitor = block
            self.spin_for_digging()

        except Exception as e:
            self.get_logger().error(f"Error collecting block: {e}")
            self.reconnect_bot()
            response.success = False
            return response

        self.get_logger().info(f"collected block: {vec}")

        response.success = True
        return response

    def inventory_contents_service_callback(self, request: Inventory.Request, response: Inventory.Response):
        items = bot.inventory.items()
        inventory = []

        self.get_logger().info(f"Inventory contents: {items}")
        for item in items:
            item_msg = Item()
            item_msg.id = item.type
            item_msg.count = item.count
            item_msg.slot = item.slot
            item_msg.metadata = item.metadata
            inventory.append(item_msg)

        response.inventory = inventory
        return response

    def place_block_callback(self, request: PlaceBlock.Request, response: PlaceBlock.Response):
        block = request.block
        mc_block = bot.blockAt(Vec3(
            block.block_pose.position.x, block.block_pose.position.y, block.block_pose.position.z))
        if mc_block.name == 'air':
            self.get_logger().info(f"Can't place block on air")
            response.success = False
            return response

        response.success = self.place_block(block)
        return response

    def place_block(self, block: BlockPose):
        self.get_logger().info(f"Placing {block} ")

        item: Item = block.block
        block_pose: Pose = block.block_pose
        point: Point = block_pose.position
        face_vector: Point = block.face_vector
        point = Vec3(point.x, point.y, point.z)
        face_vector = Vec3(face_vector.x, face_vector.y, face_vector.z)

        self.get_logger().info(
            f"Placing block: {item.id} at {point} with {face_vector}")
        block = bot.blockAt(point)

        # Get to block
        bot.pathfinder.setGoal(None)
        goal = pathfinder.goals.GoalPlaceBlock(block.position.plus(
            face_vector), bot.world, {'range': 5, 'half': 'top'})
        bot.pathfinder.setGoal(goal)
        self.spin_for_goal()

        items = bot.inventory.items()
        item = list(filter(lambda i: i.type == item.id, items))
        if len(item) == 0:
            self.get_logger().info(f"Can't place block: {item.id}")
            return False
        item = item[0]
        bot.equip(item, 'hand')

        c = 0
        while 1:
            try:
                bot.placeBlock(block, face_vector)
                self.get_logger().info(
                    f"Placed block: {item.id} at {point} with {face_vector}")

                return True
            except Exception as e:
                time.sleep(0.1)
                self.get_logger().info(f"{e}")
                c += 1
                if c >= 10:
                    return False

    def craft_item_service_callback(self, request: Craft.Request, response: Craft.Response):
        item = request.item
        self.get_logger().info(f"Crafting item: {item.id}")

        if request.crafting_table:
            crafting_table = bot.blockAt(Vec3(
                request.crafting_table_location.x, request.crafting_table_location.y, request.crafting_table_location.z))
        else:
            crafting_table = None  # None is an alias for the player's inventory

        recipe = list(bot.recipesFor(item.id, None, None, crafting_table))

        self.get_logger().info(f"Recipe: {recipe}")
        if len(recipe) == 0:
            self.get_logger().info(f"Can't craft item: {item.id}")
            response.success = False
            return response

        bot.craft(recipe[0], item.count, crafting_table)
        time.sleep(0.05)
        self.get_logger().info(f"Crafted item: {item.id}")
        response.success = True

        return response

    # TODO use stove

    # TODO kill or avoid mobs

    def local_pose_timer_callback(self):
        try:
            bot_position = bot.entity.position
        except Exception as e:
            self.get_logger().info(f"Error getting bot position: {e}")
            return

        pose = PoseStamped()
        pose.pose.position.x = float(bot_position.x)
        pose.pose.position.y = float(bot_position.y)
        pose.pose.position.z = float(bot_position.z)
        pose.header.stamp = self.get_clock().now().to_msg()
        self.local_position_publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = MinerosMain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
