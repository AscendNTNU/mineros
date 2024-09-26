const rclnodejs = require('rclnodejs')
const mineflayer = require('mineflayer')
const {
  pathfinder,
  Movements,
  goals: { GoalNearXZ, GoalNear, GoalLookAtBlock, GoalPlaceBlock }
} = require('mineflayer-pathfinder')
const toolPlugin = require('mineflayer-tool').plugin
const collectBlock = require('mineflayer-collectblock').plugin
const Vec3 = require('vec3').Vec3

// === // Configuring bot // === //
const bot = mineflayer.createBot({
  host: 'localhost',
  port: 25565,
  username: 'MinerosBot',
  hideErrors: false
})

console.log('Bot created')
bot.loadPlugin(pathfinder)
bot.loadPlugin(toolPlugin)
bot.loadPlugin(collectBlock)
bot.loadPlugin(toolPlugin)

let registry = null

bot.once('spawn', () => {
  bot.chat('Hello, I am MinerosBot')
  const defaultMove = new Movements(bot)
  defaultMove.allow1by1towers = false
  bot.pathfinder.setMovements(defaultMove)
  bot.pathfinder.thinkTimeout = 10000
  registry = require('prismarine-registry')(bot.version)

  main()
})

// bot.on('goal_reached', (goal) => {

// })

// === // Ros Node // === //

// const ParameterType = rclnodejs.ParameterType
// const Parameter = rclnodejs.Parameter
// const ParameterDescriptor = rclnodejs.ParameterDescriptor

class MinerosBot {
  constructor () {
    this.node = new rclnodejs.Node('mineros_js_node')
    this.clock = new rclnodejs.Clock()
    this.positionReached = false

    // State publishers
    this.localPositionPublisher = this.node.createPublisher(
      'geometry_msgs/msg/PoseStamped',
      '/mineros/local_position/pose',
      { qos: 10 }
    )

    this.localPositionTimer = this.node.createTimer(100, () => {
      let poseStampedMsg = rclnodejs.createMessageObject(
        'geometry_msgs/msg/PoseStamped'
      )

      let botPosition = bot.entity.position
      poseStampedMsg.pose.position.x = botPosition.x
      poseStampedMsg.pose.position.y = botPosition.y
      poseStampedMsg.pose.position.z = botPosition.z
      poseStampedMsg.header.stamp = this.clock.now().toMsg()
      this.localPositionPublisher.publish(poseStampedMsg)
    })

    // Movement
    this.findYService = this.node.createService(
      'mineros_inter/srv/BlockInfo',
      '/mineros/findy',
      this.findYCallback
    )

    this.setPositionService = this.node.createService(
      'mineros_inter/srv/MoveTo',
      '/mineros/set_position',
      this.setPositionCallback
    )

    this.lookAtBlockService = this.node.createService(
      'mineros_inter/srv/MoveTo',
      '/mineros/look_at_block',
      this.lookAtBlockCallback
    )

    // Mining Control
    this.findBlocksService = this.node.createService(
      'mineros_inter/srv/FindBlocks',
      '/mineros/mining/find_blocks',
      this.findBlocksCallback
    )

    this.mineBlockService = this.node.createService(
      'mineros_inter/srv/MineBlock',
      '/mineros/mining/mine_block',
      this.mineBlockCallback
    )

    this.inventoryService = this.node.createService(
      'mineros_inter/srv/Inventory',
      '/mineros/inventory/contents',
      this.inventoryCallback
    )

    // Place Block
    this.placeBlockService = this.node.createService(
      'mineros_inter/srv/PlaceBlock',
      '/mineros/interaction/place_block',
      this.placeBlockCallback
    )

    // Crafting

    this.craftItemService = this.node.createService(
      'mineros_inter/srv/Craft',
      '/mineros/interaction/craft',
      this.craftItemCallback
    )

    this.getRecipeService = this.node.createService(
      'mineros_inter/srv/Recipe',
      '/mineros/interaction/recipe',
      this.getRecipeCallback
    )

    // Furnace
    this.furnaceInfoService = this.node.createService(
      'mineros_inter/srv/FurnaceInfo',
      '/mineros/interaction/furnace_info',
      this.furnaceInfoCallback
    )

    this.furnaceUpdateService = this.node.createService(
      'mineros_inter/srv/FurnaceUpdate',
      '/mineros/interaction/furnace_update',
      this.furnaceUpdateCallback
    )

    // Spatial Awareness
    this.blockInfoService = this.node.createService(
      'mineros_inter/srv/BlockInfo',
      '/mineros/blockinfo',
      this.findBlockInfoCallback
    )
  }

  /**
   * Returns the ground level y coordinate for a given x and z coordinate
   * Usefull for navigation by allowing user to treat the search space as 2D
   *
   * Iterates up or down from the current bot position depending on whether it is air or not
   *  */
  async findYCallback (request, response) {
    console.log('find y callback')
    let result = response.template

    result.block.block_pose.position.x = request.block_pose.position.x
    result.block.block_pose.position.z = request.block_pose.position.z

    let botPosition = bot.entity.position
    let blockAtBotHeight = bot.blockAt(
      new Vec3(
        request.block_pose.position.x,
        botPosition.y,
        request.block_pose.position.z
      )
    )

    let iteration = 1
    if (blockAtBotHeight.type == 0) {
      iteration = -1
    }
    let block = blockAtBotHeight

    while (true) {
      let previousBlock = block
      let newPose = new Vec3(
        request.block_pose.position.x,
        previousBlock.position.y + iteration,
        request.block_pose.position.z
      )

      block = bot.blockAt(newPose)
      if (block.type == 0 && previousBlock.type != 0) {
        // Iterated from a block to air
        result.block.block_pose.position.y = parseFloat(
          previousBlock.position.y
        )
        response.send(result)
        return
      } else if (block.type != 0 && previousBlock.type == 0) {
        // Iterated from air to a block
        result.block.block_pose.position.y = parseFloat(block.position.y)
        response.send(result)
        return
      }
    }
  }

  async findBlockInfoCallback (request, response) {
    let result = response.template

    let block_pose = new Vec3(
      request.block_pose.position.x,
      request.block_pose.position.y,
      request.block_pose.position.z
    )
    let block = bot.blockAt(block_pose)

    result.block.block.id = block.type
    result.block.block.count = 1
    result.block.block.display_name = block.displayName

    result.block.block_pose.position.x = block_pose.x
    result.block.block_pose.position.y = block_pose.y
    result.block.block_pose.position.z = block_pose.z

    response.send(result)
    return
  }

  async setPositionCallback (request, response) {
    console.log(
      'Movind to: ' +
        request.pose.pose.position.x +
        ' ' +
        request.pose.pose.position.y +
        ' ' +
        request.pose.pose.position.z
    )

    // Clear bots goal
    let result = response.template

    bot.pathfinder.setGoal(null)

    let goal = null
    if (request.pose.pose.position.y == -1.0) {
      goal = new GoalNearXZ(
        request.pose.pose.position.x,
        request.pose.pose.position.z,
        1
      )
    } else {
      goal = new GoalNear(
        request.pose.pose.position.x,
        request.pose.pose.position.y,
        request.pose.pose.position.z,
        1
      )
    }

    // If the promise resolves return true else return false
    try {
      await bot.pathfinder.goto(goal)
      result.success = true
    } catch (error) {
      result.success = false
    }
    response.send(result)
  }

  async lookAtBlockCallback (request, response) {
    let result = response.template

    bot.pathfinder.setGoal(null) // Clear bot goal
    let block_pose = new Vec3(
      request.pose.pose.position.x,
      request.pose.pose.position.y,
      request.pose.pose.position.z
    )
    let block = bot.blockAt(block_pose)
    console.log('Looking at block: ' + block.displayName)

    let goal = new GoalLookAtBlock(block.position, bot.world)

    try {
      await bot.pathfinder.goto(goal)
      result.success = true
    } catch (error) {
      result.success = false
    }
    response.send(result)
  }

  findBlocksCallback (request, response) {
    console.log('Find Blocks callback')

    let result = response.template
    let options = {
      matching: request.blockid,
      maxDistance: request.max_distance,
      count: request.count
    }

    let blocks = bot.findBlocks(options)

    let poseArrayMsg = rclnodejs.createMessageObject(
      'geometry_msgs/msg/PoseArray'
    )
    let poses = []
    for (let i = 0; i < blocks.length; i++) {
      let block_pose = blocks[i]
      let poseMsg = rclnodejs.createMessageObject('geometry_msgs/msg/Pose')
      poseMsg.position.x = block_pose.x
      poseMsg.position.y = block_pose.y
      poseMsg.position.z = block_pose.z
      poses.push(poseMsg)
    }
    poseArrayMsg.poses = poses
    result.blocks = poseArrayMsg

    response.send(result)
  }

  async mineBlockCallback (request, response) {
    let result = response.template

    let block = bot.blockAt(
      new Vec3(
        request.block.position.x,
        request.block.position.y,
        request.block.position.z
      )
    )
    console.log('Mining block: ' + block.displayName)

    // Check if the block is mineable
    await bot.tool.equipForBlock(block)
    let heldItem = bot.heldItem
    let heldItemType = null
    if (heldItem != null) {
      heldItemType = heldItem.type
    }

    if (!block.canHarvest(heldItemType)) {
      console.log(
        'Cannot harvest block: ' +
          block.displayName +
          ' with ' +
          heldItem.displayName
      )

      result.success = false
      response.send(result)
      return
    }

    // get To Block
    bot.pathfinder.setGoal(null)
    let goal = new GoalLookAtBlock(block.position, bot.world)
    await bot.pathfinder.goto(goal)

    // Check if the bot is close enough to the block to dig it
    await bot.tool.equipForBlock(block) // pathfinder may have changed the tool
    if (!bot.canDigBlock(block)) {
      console.log('Cannot dig block: ' + block.displayName)
      result.success = false
      response.send(result)
      return
    }

    // Dig the block
    try {
      await bot.dig(block)
    } catch (error) {
      console.log('Error collecting block: ' + block.displayName)
      result.success = false
      response.send(result)
      return
    }

    // sleep to let the block drop
    await new Promise(resolve => setTimeout(resolve, 200))

    // Collect the item
    let nearbyentities = bot.entities
    let entities = Object.values(nearbyentities)
    // console.log(entities)

    for (let i = 0; i < entities.length; i++) {
      let entity = entities[i]
      if (entity.name == 'item') {
        if (entity.position.distanceTo(bot.entity.position) < 10) {
          // Goto the entity position
          bot.pathfinder.setGoal(null)
          let goal = new GoalNear(
            entity.position.x,
            entity.position.y,
            entity.position.z,
            0.5
          )
          try {
            await bot.pathfinder.goto(goal)
          } catch (error) {
            result.success = false
            response.send(result)
            return
          }
        }
      }
    }

    result.success = true
    response.send(result)
  }

  inventoryCallback (request, response) {
    console.log('Inventory Callback')
    let result = response.template

    let inventory = bot.inventory.items()
    for (let i = 0; i < inventory.length; i++) {
      let item = inventory[i]
      let itemMsg = itemToItemMsg(item)
      result.inventory.push(itemMsg)
    }

    response.send(result)
  }

  async placeBlockCallback (request, response) {
    let result = response.template

    let block_pose = new Vec3(
      request.block.block_pose.position.x,
      request.block.block_pose.position.y,
      request.block.block_pose.position.z
    )
    let block = bot.blockAt(block_pose)
    console.log('Placing block: ' + block.displayName)

    if (block.name == 'air') {
      console.log('Cannot place block on air block')
      result.success = false
      response.send(result)
      return
    }

    let item = request.block.block
    let faceVector = new Vec3(
      request.block.face_vector.x,
      request.block.face_vector.y,
      request.block.face_vector.z
    )

    // Check if bot has the item in inventory
    let inventory = bot.inventory.items()
    let mcItem = inventory.find(invItem => invItem.type == item.id)

    if (mcItem == null) {
      console.log('Bot does not have item in inventory')
      result.success = false
      response.send(result)
      return
    }

    // Get to block
    bot.pathfinder.setGoal(null)
    let goal = new GoalPlaceBlock(block.position.plus(faceVector), bot.world, {
      range: 4,
      half: 'top'
    })
    try {
      await bot.pathfinder.goto(goal)
    } catch (error) {
      console.log('Error getting to block  to place block')
      result.success = false
      response.send(result)
      return
    }

    // Place the block
    try {
      await bot.equip(mcItem, 'hand')
      await bot.placeBlock(block, faceVector)
    } catch (error) {
      console.log('Error placing block')
      result.success = false
      response.send(result)
      return
    }

    result.success = true
    response.send(result)
  }

  async craftItemCallback (request, response) {
    console.log('Crafting: ' + request.item.id)
    let result = response.template

    let craftingTable = null
    if (request.crafting_table) {
      craftingTable = bot.blockAt(
        new Vec3(
          request.crafting_table_location.position.x,
          request.crafting_table_location.position.y,
          request.crafting_table_location.position.z
        )
      )
    }

    let recipes = bot.recipesAll(request.item.id, null, craftingTable)
    if (recipes.length == 0) {
      result.success = false
      response.send(result)
      return
    }

    for (let i = 0; i < recipes.length; i++) {
      try {
        await bot.craft(recipes[i], request.item.count, craftingTable)
        result.success = true
        response.send(result)
        return
      } catch (error) {
        result.success = false
      }
    }
  }

  getRecipeCallback (request, response) {
    console.log('Get Recipe Callback')
    let result = response.template

    let craftingTable = null
    if (request.crafting_table) {
      craftingTable = bot.blockAt(
        new Vec3(
          request.crafting_table_location.position.x,
          request.crafting_table_location.position.y,
          request.crafting_table_location.position.z
        )
      )
    }

    let recipe = bot.recipesAll(request.item.id, null, craftingTable)

    let returnedRecipes = []
    for (let i = 0; i < recipe.length; i++) {
      let recipeMsg = recipeToRecipeMsg(recipe[i])
      returnedRecipes.push(recipeMsg)
    }

    result.recipes = returnedRecipes
    result.success = returnedRecipes.length > 0
    response.send(result)
  }

  async furnaceInfoCallback (request, response) {
    let result = response.template
    console.log('Furnace Info Callback')

    let furnaceBlock = bot.blockAt(
      new Vec3(
        request.block_pose.position.x,
        request.block_pose.position.y,
        request.block_pose.position.z
      )
    )

    if (furnaceBlock == null) {
      console.log('Furnace block not found')
      result.success = false
      response.send(result)
      return
    } else if (furnaceBlock.type != registry.blocksByName.furnace.id) {
      console.log('Block is not a furnace, it is: ' + furnaceBlock.displayName)
      result.success = false
      response.send(result)
      return
    }

    // Look at furnace
    bot.pathfinder.setGoal(null)
    let goal = new GoalLookAtBlock(furnaceBlock.position, bot.world)

    try {
      await bot.pathfinder.goto(goal)
    } catch (error) {
      console.log('Error looking at furnace')
      result.success = false
      response.send(result)
      return
    }

    console.log('Opening furnace')
    let furnacePromise = bot.openFurnace(furnaceBlock)

    furnacePromise
      .then(furnace => {
        let inputItem = itemToItemMsg(furnace.inputItem)
        let fuelItem = itemToItemMsg(furnace.fuelItem)
        let outputItem = itemToItemMsg(furnace.outputItem)

        result.furnace.input_item = inputItem
        result.furnace.fuel_item = fuelItem
        result.furnace.output_item = outputItem
        result.success = true
        response.send(result)
      })
      .catch(error => {
        console.log('Error opening furnace')
        result.success = false
        response.send(result)
      })
  }

  furnaceUpdateCallback (request, response) {
    let result = response.template
    console.log('Furnace Update Callback')

    let furnaceBlock = bot.blockAt(
      new Vec3(
        request.block_pose.position.x,
        request.block_pose.position.y,
        request.block_pose.position.z
      )
    )

    if (furnaceBlock == null) {
      console.log('Furnace block not found')
      result.success = false
      response.send(result)
      return
    } else if (furnaceBlock.type != registry.blocksByName.furnace.id) {
      console.log('Block is not a furnace, it is: ' + furnaceBlock.displayName)
      result.success = false
      response.send(result)
      return
    }

    let furnacePromise = bot.openFurnace(furnaceBlock)
    furnacePromise
      .then(furnace => {
        if (furnace == null) {
          console.log('Error opening furnace')
          result.success = false
          response.send(result)
          return
        }

        let oldInputItem = furnace.inputItem()
        let oldFuelItem = furnace.fuelItem()
        let oldOutputItem = furnace.outputItem()

        evaluateFurnaceAction(
          request.furnace.input_item,
          oldInputItem,
          furnace.takeInput,
          furnace.putInput
        )

        evaluateFurnaceAction(
          request.furnace.output_item,
          oldOutputItem,
          furnace.takeOutput,
          null
        )

        if (!request.ignore_fuel) {
          evaluateFurnaceAction(
            request.furnace.fuel_item,
            oldFuelItem,
            furnace.takeFuel,
            furnace.putFuel
          )
        }
      })
      .catch(error => {
        console.log('Error opening furnace')
        result.success = false
        response.send(result)
        return
      })

    result.success = true
    response.send(result)
    return
  }
}

// Util functions
function itemToItemMsg (item) {
  let itemMsg = rclnodejs.createMessageObject('mineros_inter/msg/Item')
  if (item == null) {
    return itemMsg
  }

  itemMsg.id = item.type
  itemMsg.count = item.count
  itemMsg.slot = item.slot
  itemMsg.metadata = item.metadata
  itemMsg.display_name = item.displayName
  return itemMsg
}

function recipeItemToItemMsg (recipeItem) {
  let itemMsg = rclnodejs.createMessageObject('mineros_inter/msg/Item')
  if (recipeItem == null) {
    return
  }

  itemMsg.id = recipeItem.id
  itemMsg.count = recipeItem.count
  itemMsg.slot = recipeItem.slot
  itemMsg.metadata = recipeItem.metadata
  itemMsg.display_name = recipeItem.displayName
  return itemMsg
}

function recipeToRecipeMsg (recipe) {
  let recipeMsg = rclnodejs.createMessageObject('mineros_inter/msg/Recipe')
  recipeMsg.output_item = recipeItemToItemMsg(recipe.result)
  recipeMsg.input_items = []
  let items = []

  for (let row = 0; row < recipe.inShape.length; row++) {
    let rowItems = recipe.inShape[row]
    for (let col = 0; col < rowItems.length; col++) {
      let item = rowItems[col]
      items.push(item)
    }
  }

  const itemIds = items.map(item => item.id)
  const uniqueItemIds = [...new Set(itemIds)]

  for (let i = 0; i < uniqueItemIds.length; i++) {
    if (uniqueItemIds[i] == -1) {
      continue
    }
    let itemMsg = rclnodejs.createMessageObject('mineros_inter/msg/Item')
    itemMsg.id = uniqueItemIds[i]
    itemMsg.count = itemIds.filter(id => id == uniqueItemIds[i]).length
    itemMsg.metadata = 0
    itemMsg.display_name = items.find(
      item => item.id == uniqueItemIds[i]
    ).displayName
    recipeMsg.input_items.push(itemMsg)
  }

  return recipeMsg
}

function itemEquals (itemMsg, mcItem) {
  return itemMsg.id == mcItem.type && itemMsg.count == mcItem.count
}

function evaluateFurnaceAction (requestItem, oldItem, takeAction, putAction) {
  if (requestItem != null && !itemEquals(requestItem, oldItem)) {
    if (oldItem.count > 0) {
      takeAction()
    }
    if (requestItem.count > 0) {
      if (putAction == null) {
        console.log('Put action is null')
        return
      }
      putAction(requestItem.id, null, requestItem.count)
    }
  }
}

async function main () {
  await rclnodejs.init()
  const minerosBot = new MinerosBot()

  minerosBot.node.spin()
}
