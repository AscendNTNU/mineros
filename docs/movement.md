# Move to one position

To move to one position call the service `/mineros/set_position` with the service type `mineros_inter/srv/MoveTo`. The service returns true once the position is reached, If the move to command fails it returns false.


# Move to and look at block (x, y, z)
To move to look at a block call the service `/mineros/look_at_block` with the service type `mineros_inter/srv/MoveTo`. The service will move the bot to look at the block at x,y,z. Usefull when attempting to mine or place a block. The service returns true once the position is reached, If the move to command fails it returns false.

# Service Definition

## MoveTo
```
geometry_msgs/PoseStamped pose

---

bool success
```

# Dimensions of motion

## 3d
To move the bot in 3 dimensions specify values for `x`, `y` and `z` in the `geometry_msgs/Point` data type.

## 2d
By 2d its meant that the bot will move in the x and z plane, ignoring the alltitude of the bot, essentially this means that the bot will move at ground level (never go underground). To activate 2 dimensional motion specify values for `x` and `z` and set `y` to `-1.0`. in the `geometry_msgs/Point` data type.

