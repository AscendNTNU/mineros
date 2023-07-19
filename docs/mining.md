# Find blocks

call the `/mineros/mining/find_blocks` service with the FindBlocks request.

The request has the following params:
-  blockid: the id of the block, use mineflayer mc data plugin to get the id 
- max_distance: the radius to search
- count: the desired amount of blocks to find

returns a geometry_msgs/PoseArray with the blocks found

# Collect blocks
call the `/mineros/mining/mine_blocks` service with the MineBlocks service request. The request has the following params:
- blocks: a geometry_msgs/PoseArray with the blocks to mine

returns a std_msgs/Bool with the result of the operation

# Inventory
call the `/mineros/inventory/contents` service with the Inventory service request. The request is a trigger and returns:
`mineros_interfaces/Item[]` 
