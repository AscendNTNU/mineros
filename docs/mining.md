# Find blocks
call the `/mineros/mining/find_blocks` service with the FindBlocks request. The request has the following params:
-  `uint16 blockid` - the id of the block, use mineflayer mc data plugin to get the id 
- `uint16 max_distance` - the radius to search
- `uint16 count` the desired amount of blocks to find

returns:
- `geometry_msgs/PoseArray blocks` - Array of matching blocks found

# Mine block
call the `/mineros/mining/mine_block` service with the MineBlock service request. The request has the following params:
- `geometry_msgs/Pose block` - The pose of the block to mine
returns:
- `bool success` - Whether the operation was successful or not

# Inventory
call the `/mineros/inventory/contents` service with the Inventory service request. The request is a trigger (no params) and returns:
- `mineros_interfaces/Item[]` - Array of items in the inventory, see below for the item message definition

## Item msg
- `uint16 id` - The id of the item, IMPORTANT: this is the item id not the block id, they are different
- `uint16 count` - The amount of items
- `uint16 slot` - The slot in the inventory where the item is located
- `uint16 metadata` - The metadata of the item, this is used to specify the damage of a tool or the color of wool etc. If you dont know what this is just set it to 0. Just generaly ignore it unless you are a minecraft expert.
