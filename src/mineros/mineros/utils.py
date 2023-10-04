from mineros_inter.msg import Item, Furnace, Recipe


def recipe_to_recipe_msg(recipe) -> Recipe:
    def recipe_item_to_item_msg(item) -> Item:
        if item is None:
            item_msg = Item()
            item_msg.id = 0
            item_msg.count = 0
            item_msg.metadata = 0
            return item_msg

        item_msg = Item()
        item_msg.id = int(item.id)
        item_msg.count = int(item.count)
        item_msg.metadata = 0
        return item_msg

    recipe_msg = Recipe()
    recipe_msg.output_item = recipe_item_to_item_msg(recipe.result)
    
    recipe_msg.input_items = []
    items = []
    items_in_shape = [items.extend(row) for row in recipe.inShape]
    item_ids = list(map(lambda item: item.id, items))
    unique_item_ids = set(item_ids)

    for unique_item_id in unique_item_ids:
        if unique_item_id == -1:
            continue
        item_msg = Item()
        item_msg.id = int(unique_item_id)
        item_msg.count = item_ids.count(unique_item_id)
        item_msg.metadata = 0
        recipe_msg.input_items.append(item_msg)

    return recipe_msg


def item_to_item_msg(item) -> Item:
    if item is None:
        item_msg = Item()
        item_msg.id = 0
        item_msg.count = 0
        item_msg.slot = 0
        item_msg.metadata = 0
        return item_msg

    item_msg = Item()
    item_msg.id = item.type
    item_msg.count = item.count
    item_msg.slot = item.slot
    item_msg.metadata = item.metadata
    return item_msg


def item_equal(item1: Item, item2: Item):

    if item1 is None and item2 is None:
        return True
    if item1 is None or item2 is None:
        return False
    return item1.id == item2.id and item1.count == item2.count
