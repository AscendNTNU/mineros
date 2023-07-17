# Find blocks

call the `/mineros/mining/find_blocks` service with the FindBlocks request.

The request has the following params:
-  blockid: the id of the block, use mineflayer mc data plugin to get the id 
- max_distance: the radius to search
- count: the desired amount of blocks to find

returns a geometry_msgs/PoseArray with the blocks found