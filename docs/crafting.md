## Craft item
call `/mineros/interaction/craft` with a `mineros_interfaces/Craft` request. The request has the following params:
- `mineros_interfaces/Item` item to craft
- `bool crafting_table` if the bot should use a crafting table or not
- `geometry_msgs/Pose` 