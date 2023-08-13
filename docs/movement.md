# Move to one position

To move to one position publish a `geometry_msgs/PoseStamped` message to the topic `/mineros/set_position`. This will cause to bot to move to the specified position.

# Move to multiple positions

To move to multiple positions after eachother publish a `geometry_msgs/PoseArray` message to the topic `/mineros/set_position/composite`. This will cause to bot to move to the specified positions in order.

# Find y corrdinate of ground floor for a position given x and z
call `/mineros/findy` service with `mineros_interfaces/BlockInfo` message with the x and z coordinates set. The service will return a `geometry_msgs/Pose` message with all cordinates of the ground floor of the x and z position

# Dimensions of motion

## 3d
To move the bot in 3 dimensions specify values for `x`, `y` and `z` in the `geometry_msgs/Point` data type.

## 2d
By 2d its meant that the bot will move in the x and z plane, ignoring the alltitude of the bot, essentially this means that the bot will move at ground level (never go underground). To activate 2 dimensional motion specify values for `x` and `z` and set `y` to `-1.0`. in the `geometry_msgs/Point` data type.
