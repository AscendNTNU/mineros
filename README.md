# MineROS relase 1.0 documentation

MineROS allows the user to fully control a mc bot using ros services and topics, The current version of MineROS is made for ROS2 Foxy and is still in development. The purpose of this project is to give a conducive learning environment for people who want to learn ROS2 and python. Specifically for people who want to learn mavros, as this is what the project attempts to mock.

## Dependencies
- ROS2 Humble
- Python3
- Javascript
- Nodejs version >= 20
- Mineflayer Api
- Minecraft version = 1.20

## Installation

### Install correct minecraft version
Google this yourself

### Install javascript deps
``` pip install javascript ```

### Install nodejs
``` sudo apt install nodejs ```
``` sudo apt install npm ```

### Install npm packages
``` npm install --save mineflayer-collectblock ```


## Running
Make sure a minecraft server is running on localhost:25565
``` ros2 run mineros mineros_main ```

___
# API doc

Following is general api information and links to api docs.

## Minecraft data
```
IMPORTANT: BLOCKS DO NOT HAVE THE SAME IDS AS ITEMS
```

### Block ids
Several of the services and topics require knowledge of the minecraft block ids, this can be found here REMEBER TO SELECT THE CORRECT MINECRAFT VERSION: 1.20 http://prismarinejs.github.io/minecraft-data/?d=blocks

### Item ids
on the boot up of the mineros system all item ids are written to the file ` docs/items.txt `. Note that blocks dont have the same id as items


## Links to API docs
- [Movement](docs/movement.md)
- [Navigation](docs/navigation.md)
- [Mining](docs/mining.md)
- [Building](docs/building.md)
- [Furnace](docs/furnace.md)
- [Crafting](docs/crafting.md)