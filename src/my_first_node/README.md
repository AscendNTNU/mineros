# Task 1

The helper node has the goal of alleviating the fsm_node from listening to the position of the bot. This node is already set up to track the position of the bot. The task is to create a service in the helper node that allows the FSM to call the service and get the position. To do this requires 2 things: 

- 1: Create your own service in the mineros_inter package, and use it as the service definition for your service the files and directories you might want to look into are:
    - `src/mineros_inter/srv`
    - `src/mineros_inter/srv/Inventory.srv`
    - `src/mineros_inter/srv/BlockInfo.srv`
    - `src/mineros_inter/CMakeLists.txt`

- 2: Use the service definition to create your service

# Task 2

Implement a client in the fsm node that allows you to call for the bots position.

# Task 3

Implement the publisher towards mineros in the fsm that allows you to set the goal position of the bot

# Task 4

Implement logic for moving your bot around the world

# Task 5 (Optional)

Implement a launch file that allows you to launch both nodes using a single command
___

# Resources
good resources to help you complete this tutorial are:
- Looking through the source code of mineros, especially the tests
- [ROS2 Publisher Subscriber guide](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)


