const rclnodejs = require('rclnodejs')
const mineflayer = require('mineflayer')
const {
  pathfinder,
  Movements,
  goals: { GoalNearXZ, GoalNear, GoalLookAtBlock }
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

bot.once('spawn', () => {
  bot.chat('Hello, I am MinerosBot')
  const defaultMove = new Movements(bot)
  bot.pathfinder.setMovements(defaultMove)
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

    let block_pos = new Vec3(
      request.block_pose.position.x,
      request.block_pose.position.y,
      request.block_pose.position.z
    )
    let block = bot.blockAt(block_pos)

    result.block.block.id = block.type
    result.block.block.count = 1
    result.block.block.display_name = block.displayName

    result.block.block_pose.position.x = block_pos.x
    result.block.block_pose.position.y = block_pos.y
    result.block.block_pose.position.z = block_pos.z

    response.send(result)
    return
  }

  async setPositionCallback (request, response) {
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
    let block_pos = new Vec3(
      request.pose.pose.position.x,
      request.pose.pose.position.y,
      request.pose.pose.position.z
    )
    let block = bot.blockAt(block_pos)
    let goal = new GoalLookAtBlock(block.position, bot.world)

    try {
      await bot.pathfinder.goto(goal)
      result.success = true
    } catch (error) {
      result.success = false
    }
    response.send(result)
  }
}

async function main () {
  await rclnodejs.init()
  const minerosBot = new MinerosBot()

  minerosBot.node.spin()
}
