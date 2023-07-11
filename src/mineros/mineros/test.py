import time
import threading
from javascript import require, On, Once, AsyncTask, once, off
mineflayer = require('mineflayer')
pathfinder = require('mineflayer-pathfinder')
mineflayerViewer = require('prismarine-viewer').mineflayer


BOT_USERNAME = f'MinerosBot'
LAN_PORT = 25565

bot = mineflayer.createBot(
    {'host': 'localhost', 'port': LAN_PORT, 'username': BOT_USERNAME, 'hideErrors': False})
bot.loadPlugin(pathfinder.pathfinder)

mineflayerViewer(bot, { 'port': 3000, 'firstPerson': False })

# The spawn event
once(bot, 'login')
bot.chat('I spawned')

# Instantiating pathfinding
movements = pathfinder.Movements(bot)
bot.pathfinder.setMovements(movements)

print(f'Bot: {bot.username} spawned at {bot.entity.position}')


def post_position():
    global bot
    print(f'Bot: {bot.username} is at {bot.entity.position}')
    time.sleep(1)
    post_position()
    
threading.Thread(target=post_position, daemon=True).start()

bot.pathfinder.setGoal(pathfinder.goals.GoalNear(0, 0, 0, 1))

while True:
    pass
