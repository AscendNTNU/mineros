import math
from javascript import require, On, Once, AsyncTask, once, off
from mavros_msgs.srv import WaypointPush
import time
import threading
from typing import List, Tuple

mineflayer = require('mineflayer')
pathfinder = require('mineflayer-pathfinder')

position = None

def set_goal(bot, goal: Tuple[float, float, float], goal_acceptance: float):
    bot.pathfinder.setGoal(pathfinder.goals.GoalNear(
            goal[0], goal[1], goal[2], goal_acceptance))

def distance_from_waypoint(waypoint: Tuple[float, float, float]):
    x1, y1, z1 = waypoint
    x2, y2, z2 = position.x, position.y, position.z
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
      
def update_position(bot):
    global position
    position = bot.entity.position
    threading.Timer(0.1, update_position, [bot]).start()

def handle_mission_push(bot, mission_list: List[Tuple[float, float, float]], goal_acceptance: float):
    global position
    position = bot.entity.position
    threading.Timer(0.1, update_position, [bot]).start()
    
    for waypoint in mission_list:
        set_goal(bot, waypoint, goal_acceptance)
        while distance_from_waypoint(waypoint) > goal_acceptance:
            time.sleep(0.1)

    # When this is reached tha bot shouldve traveled the waypoiny path
