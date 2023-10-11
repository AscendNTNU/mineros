# Bot position

The bots position is published to `/mineros/local_position/pose` as a `geometry_msgs/PoseStamped` message.

# World Info

## Find y corrdinate of ground floor for a position given x and z
call `/mineros/findy` service with `mineros_inter/BlockInfo` message with the x and z coordinates set. The service will return a `geometry_msgs/Pose` message with all cordinates of the ground floor of the x and z position

## get info about a block
call `/mineros/block_info` service with `mineros_inter/BlockInfo` message with the x, y and z coordinates set. The request has the following params:
- `geometry_msgs/Pose block_pose`: the position of the block

returns:
- `mineros_inter/BlockPose block`: the info about the block This contains an Item msg, this uses block ids as the id not item ids.