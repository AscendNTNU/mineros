# PlaceBlocks
Call the `/mineros/interaction/place_block` service with a mineros_inter/PlaceBlock request. The request has the following params:
- `mineros_inter/BlockPose block` - An object that specifies where to place the block. See below for how to use this message

returns: 
- `bool success` - Whether the operation was successful or not

## BlockPose
The BlockPose message is used to specify where to place a block. And what block to place. It has the following params:
- `geometry_msgs/Pose block_pose` - The pose of the block to place
- `mineros_inter/Item block` - The block to place
- `geometry_msgs/Point face_vector` - The vector that specifies which face of the block to place on

It is very important that the block param is specified using the item id of the block that you wish to place, not the block id.

The face vector specifies what face of the block you want to place your block on. The vector (0,1,0) (up) is for the top face. Think of this vector as the normal vector to the face you want to place on. This is badly documented in the minecraft API if you need to place on any other face than the top face Id recommend experimenting with the face vector until you get the desired result.