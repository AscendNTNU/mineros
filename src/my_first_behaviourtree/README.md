# Building and running
1) Build the ROS packages
```shell
colcon build
source install/setup.bash
```

2) Make sure the mineros bot is launched
```shell
ros2 launch mineros-js bot.launch.py
```

3) To launch your own behaviour tree (or the test tree provided)
```shell
ros2 run my_first_behaviourtree launch
```


# Making your own behaviour tree
To make your own behaviour tree you are free to use the execution nodes that already exist in /include and /src

To make additional nodes that subscribe/publish to a topic or clinet look at the [BehaviourTree.ROS2 README](../BehaviorTree.ROS2/README.md). Specifically the ROS Behaviour Wrappers

To make non-ROS related nodes, for fek calculations etc look through the docs at https://www.behaviortree.dev/docs/Intro 
Additional actions, controls and decorator nodes can be found in the BehaviourTree.CPP git repo, however these are generally poorly documented and can be somewhat difficult to understand. However, you are free to look under [BehaviourTree.CPP/include](../BehaviorTree.CPP/include/behaviortree_cpp/) for additional nodes that you would like to use

Execution nodes are made with a headerfile in the /include directory and a corresponding src file where the implementation details are in the /src directory. (If you have any questions regarding this send me a message on slack or additionally Lars Andre worked with header files last year so you can also ask him)

All nodes you want in the tree must me added by #including them in the [launch file](launch.cpp). They also need to be registered using 
```cpp
 //For non ROS nodes
factory.registerNodeType<CurrentPosition>("node_name");
```
```cpp
// For ROS nodes
factory.registerNodeType<MoveTo>("MoveTo", BT::RosNodeParams(node, "topic_name"));
```

The tree is made in the [trees folder](trees/), pharsing can be difficult to understand during the beginning, however this is fairly well explained in the behaviour tree docs https://www.behaviortree.dev/docs/learn-the-basics/xml_format 

The goal of the intro project is to get a sense of how to setup and use behaviourtree, especially with ROS2 in order to complete a mission. The maze is very challengling, so don't be discurraged if you are unable to complete it. The main point of this project is to start learning the tools we will be using throughout the upcoming year!