from mineros_interfaces.msg import Item, Furnace

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

