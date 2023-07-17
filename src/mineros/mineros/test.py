from javascript import require, once
mineflayer = require('mineflayer')
pathfinder = require('mineflayer-pathfinder')
mineflayerViewer = require('prismarine-viewer').mineflayer


BOT_USERNAME = 'DataBot'
LAN_PORT = 25565

bot = mineflayer.createBot(
    {'host': 'localhost', 'port': LAN_PORT, 'username': BOT_USERNAME, 'hideErrors': False})
bot.loadPlugin(pathfinder.pathfinder)

mc_data = require('minecraft-data')(bot.version)


# The spawn event
once(bot, 'login')
bot.chat('I spawned')

print(mc_data.blocksByName['grass_block'].id)