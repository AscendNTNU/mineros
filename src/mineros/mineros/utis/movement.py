from javascript import require, On, Once, AsyncTask, once, off
from mavros_msgs.srv import WaypointPush
import time
import threading
from typing import List, Tuple

mineflayer = require('mineflayer')
pathfinder = require('mineflayer-pathfinder')

position = None


def handle_mission_push(bot, mission_list: List[Tuple[float, float, float]]):
    global position
    position = bot.entity.position
    threading.Timer(0.1, update_position, [bot]).start()
    
    


def update_position(bot):
    global position
    position = bot.entity.position
    threading.Timer(0.1, update_position, [bot]).start()
