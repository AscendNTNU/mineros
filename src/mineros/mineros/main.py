import rclpy
from rclpy.node import Node
from javascript import require, On, Once, AsyncTask, once, off


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
     
    
    


def main(args=None):
    rclpy.init(args=args)
    node = mineros_main_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
