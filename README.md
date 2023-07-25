# MineROS documentation

MineROS allows the user to fully control a mc bot using ros services and topics, The current version of MineROS is made for ROS2 Foxy and is still in development. The purpose of this project is to give a conducive learning environment for people who want to learn ROS2 and python. Specifically for people who want to learn mavros, as this is what the project attempts to mock.

## Dependencies
- ROS2 Foxy
- Python3
- Javascript
- Nodejs
- Mineflayer Api
- Minecraft server

## Installation

### Install javascript deps
``` pip install javascript ```

### Install nodejs
``` sudo apt install nodejs ```
``` sudo apt install npm ```

## Running
Make sure a minecraft server is running on localhost:25565
``` python3 mineros.py ```

## Minecraft data
To access the block ids and other data, use the mc data plugin for mineflayer, howto example:

```python

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
```
This requires the connecting an additional bot to the server, this bot must always be called DataBot to differentiate it from other bots
___

# Links to API docs
- [Movement](docs/movement.md)
- [Navigation](docs/navigation.md)
- [Mining](docs/mining.md)
- [Building](docs/building.md)