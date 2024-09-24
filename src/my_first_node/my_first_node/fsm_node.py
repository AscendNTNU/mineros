import time
import rclpy
from rclpy.node import Node

from typing import Any
from geometry_msgs.msg import PoseStamped
from mineros_inter.srv import BlockInfo, BotPos, MoveTo

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
        
        # This is how you print to terminal in a ros node
        self.get_logger().info('Hello World!')
        
        # This definition isnt needed for this example, it only shows how to implement a client, we wil only be using publishers
        # and subscribers for this tutorial. Implementing publishers and subscribers is similar
        self.find_y_client = self.create_client(
            BlockInfo,
            '/mineros/findy'
        )
        
        self.bot_pos_service = self.create_client(
            BotPos,
            '/mineros/get_bot_position',
        )

        self.look_at_block = self.create_client(
            MoveTo,
            '/mineros/look_at_block',
        )

        self.set_position_client = self.create_client(
            MoveTo,
            '/mineros/set_position',
        )

        self.build_up_100(0.0, 71.0, 0.0)
        
        # Task 2
        # To make the bot move in a square we need to know the bots location and we need to be able to move the bot
        # set up the required publisher and subscriber here:
        
        # while 1:
        #     while self.bot_pos_service.wait_for_service(timeout_sec=1.0) == False:
        #         self.get_logger().info('Waiting for service')
        #         request = BotPos.Request()
        #         future = self.bot_pos_service.call_async(request)
        #         rclpy.spin_until_future_complete(self, future)
        #         self.get_logger().info('Result: %s' % future.result())
        
        
    def position(self, msg: PoseStamped):
        self.position = msg
        
    def position_move(self, ps: PoseStamped):
        while self.position is None:
            self.get_logger().info('Waiting for position')
            rclpy.spin_once(self, timeout_sec=0.1)
        

        move_to_command = MoveTo.Request()
        move_to_command.pose = ps

        future = self.look_at_block.call_async(move_to_command)
        #rclpy.spin(self)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'{future.result()}')
    
    def build_up_100(self, x_cord:float, y_cord:float, z_cord:float):
        ps = PoseStamped()
        ps.pose.position.x = x_cord
        ps.pose.position.y = y_cord
        ps.pose.position.z = z_cord
        y_goal = y_cord +100.0
        for z in range(int(y_cord), int(y_goal)):
            ps.pose.position.y += 1
            ps.pose.position.x += 1
            self.position_move(ps)

        
def main(args=None):
    rclpy.init(args=args)
    node = MyFirstFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    