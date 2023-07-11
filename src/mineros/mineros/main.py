import rclpy
from rclpy.node import Node
from javascript import require, On, Once, AsyncTask, once, off

from mavros_msgs.srv import WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached

from geometry_msgs.msg import PoseStamped



class mineros_main_node(Node):
    def __init__(self):
        super().__init__('mineros_main_node')
        self.get_logger().info("Hello World!")

        mineflayer = require('mineflayer')
        
        random_number = id([]) % 1000 # Give us a random number upto 1000
        BOT_USERNAME = f'colab_{random_number}'

        bot = mineflayer.createBot({ 'host': 'localhost', 'port': '25565' , 'username': BOT_USERNAME, 'hideErrors': False })
        # The spawn event 
        once(bot, 'login')
        bot.chat('I spawned')
        
        # Mode control
        self.mode_control_topic = self.create_subscription()
        
        # Mission mode
        self.mission_push = self.create_service(
            WaypointPush,
            '/mineros/mission/push',
            self.mission_push_callback,
        )
        
        self.mission_clear_service = self.create_service(
            WaypointClear,
            '/mineros/mission/clear',
            self.mission_clear_callback
        )
        
        self.waypoint_reached_topic = self.create_publisher(
            WaypointReached,
            '/mineros/mission/reached',
            self.mission_reached_callback
        )
        
        # Offboard mode
        self.set_potision_local = self.create_subscription(
            PoseStamped,
            '/mineros/set_position/local',
            self.set_position_local_callback
        )
        
        
        
    
    
    


def main(args=None):
    rclpy.init(args=args)
    node = mineros_main_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
