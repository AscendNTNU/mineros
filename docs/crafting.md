# Craft item
call `/mineros/interaction/craft` with a `mineros_inter/Craft` request. The request has the following params:

- `mineros_inter/Item` - item to craft
- `bool crafting_table` - if the bot should use a crafting table or not
- `geometry_msgs/Pose crafting_table_location` -  The location of the crafting table to use, needs only be specified if crafting_table is true
- `bool danger_mode` - activates danger mode, danger mode gets all recipes for the item available whereas normal mode only gets the recipes that the bot has the items required to craft it. If this is used and the bot doesnt have the items required it will crash unrecoverably.

returns:
- `bool success` - Whether the operation was successful or not

# Request item
call `/mineros/interaction/request` with a `mineros_inter/Request` request. The request has the following params:
- `mineros_inter/Item` - item to request
- `bool crafting_table` - if the bot should use a crafting table or not
- `geometry_msgs/Pose crafting_table_location` -  The location of the crafting table to use, needs only be specified if crafting_table is true

returns:
- `bool success` - Whether the operation was successful or not
- `mineros_inter/Recipe[]` - The recipes it found

## Note on internals
Internally mineflayer looks up the recipe of the item and checks if the bot has the items required to craft it, if it does it crafts if not it returns false.