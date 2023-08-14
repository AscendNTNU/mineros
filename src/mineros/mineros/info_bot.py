import time
import threading
from javascript import require, On, Once, AsyncTask, once, off
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from mineros_interfaces.srv import FindBlocks, MineBlock, Inventory, PlaceBlock, Craft, BlockInfo
from mineros_interfaces.msg import Item, BlockPose

mineflayer = require('mineflayer')
pathfinder = require('mineflayer-pathfinder')
toolPlugin = require('mineflayer-tool').plugin
collectBlock = require('mineflayer-collectblock').plugin
Vec3 = require('vec3').Vec3


class InfoBot(Node):
    def __init__(self):
        super().__init__('mineros_main_node')
        self.get_logger().info("Hello World!")

        # Params
        self.declare_parameter('goal_acceptance', 1)
        self.declare_parameter('bot_username', 'MinerosInfoBot')
        self.declare_parameter('lan_port', 25565)
        self.declare_parameter('main_bot_username', 'MinerosBot')
        self.declare_parameter('follow_distance', 5)
        
        BOT_USERNAME = self.get_parameter(
            'bot_username').get_parameter_value().string_value
        LAN_PORT = self.get_parameter(
            'lan_port').get_parameter_value().integer_value
        self.main_bot_username = self.get_parameter(
            'main_bot_username').get_parameter_value().string_value
        self.goal_acceptance = self.get_parameter(
            'goal_acceptance').get_parameter_value().double_value
        self.follow_distance = self.get_parameter(
            'follow_distance').get_parameter_value().integer_value

        self.bot = mineflayer.createBot(
            {'host': 'localhost', 'port': LAN_PORT, 'username': BOT_USERNAME, 'hideErrors': False})
        self.bot.loadPlugin(pathfinder.pathfinder)
        self.bot.loadPlugin(toolPlugin)
        self.bot.loadPlugin(collectBlock)

        # The spawn event
        once(self.bot, 'login')
        self.bot.chat('I spawned')

        # Instantiating pathfinding
        self.movements = pathfinder.Movements(self.bot)
        self.bot.pathfinder.setMovements(self.movements)

        self.mission = []
        self.mission_thread = None
        self.current_position = self.bot.entity.position

        timers_cbg = ReentrantCallbackGroup()
        
        self.main_bot = None
        self.main_bot_lock = threading.Lock()

        self.main_bot_spawned_service = self.create_service(
            Trigger,
            '/mineros/info/main_bot_spawned',
            self.init_bot,
            callback_group=timers_cbg
        )

        # Info publishers
        self.local_position_publisher = self.create_publisher(
            PoseStamped,
            '/mineros/local_position/pose',
            10
        )

        self.local_pose_timer = self.create_timer(
            0.1,
            self.local_pose_timer_callback,
            callback_group=timers_cbg
        )

    def init_bot(self, request: Trigger.Request, response: Trigger.Response):
        with self.main_bot_lock:
            self.get_logger().info('main bot spawned')
            while self.main_bot is None:
                self.main_bot = self.bot.players[self.main_bot_username]
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info (f"Main bot: {self.main_bot}")
                
            
            while self.main_bot.entity is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                self.main_bot = self.bot.players[self.main_bot_username]
                self.get_logger().info (f"Main bot: {self.main_bot}")
                
                
            self.get_logger().info(f"Main bot position: {self.main_bot.entity.position}")
            goal = pathfinder.goals.GoalFollow(self.main_bot.entity, self.follow_distance)
            self.bot.pathfinder.setGoal(goal)
            
            response.success = True
            return response

    def local_pose_timer_callback(self):
        with self.main_bot_lock:
            if self.main_bot is None:
                self.get_logger().info('main bot is none')
                return
            if self.main_bot.entity is None:
                self.get_logger().info('main bot entity is none')
                return
            
            bot_position = self.bot.players[self.main_bot_username].entity.position
            # self.get_logger().info(f"Bot position: {bot_position}")
            pose = PoseStamped()
            pose.pose.position.x = float(bot_position.x)
            pose.pose.position.y = float(bot_position.y)
            pose.pose.position.z = float(bot_position.z)
            self.get_logger().info(f"Bot position: {pose.pose.position}")
            self.local_position_publisher.publish(pose)
    

def main(args=None):
    rclpy.init(args=args)
    node = InfoBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
