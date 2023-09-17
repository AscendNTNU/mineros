def item_equal(item1, item2):
    
    if item1 is None and item2 is None:
        return True
    if item1 is None or item2 is None:
        return False
    return item1.id == item2.id and item1.count == item2.count
