# Move to one position

To move to one position publish a `geometry_msgs/PoseStamped` message to the topic `/mineros/set_position`. This will cause to bot to move to the specified position.

# Move to multiple positions

To move to multiple positions after eachother publish a `geometry_msgs/PoseArray` message to the topic `/mineros/set_position/composite`. This will cause to bot to move to the specified positions in order.

# Confirmation of position reached
Published with Empty message to `/mineros/set_position/reached` topic when the bot has reached the position.

# Move to look at block (x, y, z)
To move to look at a block publish a `geometry_msgs/PoseStamped` message to the topic `'/mineros/set_look_at_block'`. This will cause to bot to move to look at the specified block position.



# Dimensions of motion

## 3d
To move the bot in 3 dimensions specify values for `x`, `y` and `z` in the `geometry_msgs/Point` data type.

## 2d
By 2d its meant that the bot will move in the x and z plane, ignoring the alltitude of the bot, essentially this means that the bot will move at ground level (never go underground). To activate 2 dimensional motion specify values for `x` and `z` and set `y` to `-1.0`. in the `geometry_msgs/Point` data type.

# Note on why a topic is used instead of a service
The reason why this is a topic and not a service is because mineros handles movement as a topic. This is because while the "psuedo service" is being handled you still need to be able to make other calls to mineros. This is not allowed yet in this relase of mineros

This is only to stay consistent with mavros, it is actually a silly design choice. This is subject to change in the future.