import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from javascript import require, On, Once, AsyncTask, once, off
from typing import List, Tuple

from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached, State

from geometry_msgs.msg import PoseStamped
from .utis.movement import handle_mission_push

mineflayer = require('mineflayer')
pathfinder = require('mineflayer-pathfinder')


class MinerosMain(Node):
    def __init__(self):
        super().__init__('mineros_main_node')
        self.get_logger().info("Hello World!")

        # Params
        self.declare_parameter('goal_acceptance', 1)
        self.declare_parameter('bot_username', 'MinerosBot')
        self.declare_parameter('lan_port', 25565)

        BOT_USERNAME = self.get_parameter(
            'bot_username').get_parameter_value().string_value
        LAN_PORT = self.get_parameter(
            'lan_port').get_parameter_value().integer_value
        self.goal_acceptance = self.get_parameter(
            'goal_acceptance').get_parameter_value().double_value

        self.bot = mineflayer.createBot(
            {'host': 'localhost', 'port': LAN_PORT, 'username': BOT_USERNAME, 'hideErrors': False})
        self.bot.loadPlugin(pathfinder.pathfinder)

        # The spawn event
        once(self.bot, 'login')
        self.bot.chat('I spawned')

        # Instantiating pathfinding
        self.movements = pathfinder.Movements(self.bot)
        self.bot.pathfinder.setMovements(self.movements)

        self.mode = 'Loiter'
        self.mission = []
        self.mission_thread = None
        self.current_position = self.bot.entity.position

        timers_cbg = ReentrantCallbackGroup()

        # Mode control
        """
        Modes: 
        - Loiter
        - Mission
        - Offboard
        """

        self.mode_control_service = self.create_service(
            SetMode,
            '/mineros/set_mode',
            self.mode_control_callback
        )

        # Mission mode
        self.mission_push = self.create_service(
            WaypointPush,
            '/mineros/mission/push',
            self.mission_push_callback,
        )

        self.waypoint_reached_topic = self.create_publisher(
            WaypointReached,
            '/mineros/mission/reached',
            10
        )

        self.mission_clear_service = self.create_service(
            WaypointClear,
            '/mineros/mission/clear',
            self.mission_clear_callback
        )

        # Offboard mode
        self.set_potision_local = self.create_subscription(
            PoseStamped,
            '/mineros/set_position/local',
            self.set_position_local_callback,
            10
        )

        # Info Publishers

        # State publishers
        self.state_topic = self.create_publisher(
            State,
            '/mineros/state',
            10
        )

        self.state_timer = self.create_timer(
            0.2,
            self.state_timer_callback,
            callback_group=timers_cbg
        )

        # Offboard mode publishers
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

        threading.Thread(target=handle_mission_push, args=(self.bot,)).start()

    def mode_control_callback(self, request: SetMode.Request, response: SetMode.Response):
        self.get_logger().info(f"Mode control: {request.custom_mode}")
        self.mode = request.custom_mode
        response.success = True
        return response

    def mission_push_callback(self, request: WaypointPush.Request, response: WaypointPush.Response):
        points: List[Waypoint] = request.waypoints
        wp = Waypoint()
        
        formatted_points: List[Tuple[float, float, float]] = list(map(lambda wp: [wp.x_lat, wp.y_long, wp.z_alt], points))
        threading.Thread(target=handle_mission_push, args=(self.bot, formatted_points)).start()
        response.success = True
        return response


    def mission_clear_callback(self, request: WaypointClear.Request, response: WaypointClear.Response):
        pass

    def set_position_local_callback(self, msg: PoseStamped):
        self.bot.pathfinder.setGoal(pathfinder.goals.GoalNear(
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, self.goal_acceptance))

    def state_timer_callback(self):
        state = State()
        state.mode = self.mode
        self.state_topic.publish(state)

    def local_pose_timer_callback(self):
        bot_position = self.bot.entity.position
        self.get_logger().info(f"Bot position: {bot_position}")

        pose = PoseStamped()
        pose.pose.position.x = float(bot_position.x)
        pose.pose.position.y = float(bot_position.y)
        pose.pose.position.z = float(bot_position.z)
        self.local_position_publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = MinerosMain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
