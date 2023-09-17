# Furnace information

To get furnace info call the `/mineros/interaction/furnace_info` service with the FurnaceInfo service request. The request has the following params:
- `geometry_msgs/Pose block_pose` - The pose of the furnace block

returns:
- `mineros_interfaces/Furnace` - The state of the furnace, see the msg definition for more info
- `bool success` - Whether the operation was successful or not

# Furnace update

To update the furnace call the `/mineros/interaction/furnace_update` service with the FurnaceUpdate service request. The request contains a Furnace msg, the philosophy of this service is that the user inputs the state it wishes to set the furnace into, and mineros will attempt this. The request has the following params:
- `mineros_interfaces/Furnace furnace` - The state of the furnace, see the msg definition for more info
- `geometry_msgs/Pose block_pose` - The pose of the furnace block
- `bool ignore_fuel` - Whether to ignore the fuel during the furnace operation or not

returns:
- `bool success` - Whether the operation was successful or not