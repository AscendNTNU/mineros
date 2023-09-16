# Find blocks

call the `/mineros/mining/find_blocks` service with the FindBlocks request.

The request has the following params:
-  blockid: the id of the block, use mineflayer mc data plugin to get the id 
- max_distance: the radius to search
- count: the desired amount of blocks to find

returns a geometry_msgs/PoseArray with the blocks found

# Collect block
call the `/mineros/mining/mine_block` service with the MineBlock service request. The request has the following params:
- block: a geometry_msgs/Pose with the block to mine

returns a std_msgs/Bool with the result of the mining operation

# Inventory
call the `/mineros/inventory/contents` service with the Inventory service request. The request is a trigger and returns:
`mineros_interfaces/Item[]` 
