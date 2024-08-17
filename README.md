# MineROS relase 1.0 documentation

MineROS allows the user to fully control a mc bot using ros services and topics, The current version of MineROS is made for ROS2 Humble and is still in development. The purpose of this project is to give a conducive learning environment for people who want to learn ROS2 and python. Specifically for people who want to learn mavros, as this is what the project attempts to mock.

## Dependencies
- ROS2 Humble
- Python3
- Nodejs version >= 20
- Mineflayer Api
- Minecraft version = 1.20

## Installation

### Install correct minecraft version
Google this yourself

### Install nodejs
``` sudo apt install nodejs ```

``` sudo apt install npm ```

Make sure that this is npm version 20. Which should be the case if you run this on ubuntu 22

### Install npm packages
``` npm install --save mineflayer-collectblock ```

``` npm npm i rclnodejs ```

### Building the mineros bot

1) Init submodules to pull the mineros interfaces
```bash
git submodule update --init --recursive
git submodule update --recursive
```

2) from root run `colcon build` to build the ros packages

3) Generate the javascript Ros message interface
```bash
cd src/mineros-js
npx generate-ros-messages
```


## Running
Make sure a minecraft server is running on localhost:25565
` ros2 launch src/mineros-js/launch/example.launch.py `

Then launch your own script to control the bot

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