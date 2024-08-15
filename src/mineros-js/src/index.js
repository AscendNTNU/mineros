const rclnodejs = require('rclnodejs');
const mineflayer = require('mineflayer');
const { pathfinder, Movements, goals: { GoalNear } } = require('mineflayer-pathfinder');
const toolPlugin = require('mineflayer-tool').plugin;
const collectBlock = require('mineflayer-collectblock').plugin;
const Vec3 = require('vec3').Vec3;

// === // Configuring bot // === //
const bot = mineflayer.createBot(
  {'host': 'localhost', 'port': 25565, 'username': 'MinerosBot', 'hideErrors': false});

console.log('Bot created');
bot.loadPlugin(pathfinder);
bot.loadPlugin(toolPlugin);
bot.loadPlugin(collectBlock);
bot.loadPlugin(toolPlugin);

let spawned = false;

bot.once('spawn', () => {
  bot.chat('Hello, I am MinerosBot');
  const defaultMove = new Movements(bot);
  bot.pathfinder.setMovements(defaultMove);
  spawned = true;

  main();
});




// === // Ros Node // === //

const ParameterType = rclnodejs.ParameterType;
const Parameter = rclnodejs.Parameter;
const ParameterDescriptor = rclnodejs.ParameterDescriptor;


async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('mineros_js_node');
  
  let findYService = node.createService('mineros_inter/srv/BlockInfo', '/mineros/findy', findYCallback);
  
  
  node.spin();
}

/**
 * Returns the ground level y coordinate for a given x and z coordinate
 * Usefull for navigation by allowing user to treat the search space as 2D
 * 
 * Iterates up or down from the current bot position depending on whether it is air or not
 *  */
async function findYCallback(request, response) {
  console.log('find y callback')
  let result = response.template;

  result.block.block_pose.position.x = request.block_pose.position.x;
  result.block.block_pose.position.z = request.block_pose.position.z;

  let botPosition = bot.entity.position;
  let blockAtBotHeight = bot.blockAt(new Vec3(request.block_pose.position.x, botPosition.y, request.block_pose.position.z));

  let iteration = 1;
  if (blockAtBotHeight.type == 0){
    iteration = -1;
  }
  let block = blockAtBotHeight;
  
  while (true) {
    let previousBlock = block;
    let newPose = new Vec3(request.block_pose.position.x, previousBlock.position.y + iteration, request.block_pose.position.z);

    block = bot.blockAt(newPose);
    if (block.type == 0 && previousBlock.type != 0) {
      // Iterated from a block to air
      result.block.block_pose.position.y = parseFloat(previousBlock.position.y);
      response.send(result);
      return;
    } else if (block.type != 0 && previousBlock.type == 0) {
      // Iterated from air to a block
      result.block.block_pose.position.y = parseFloat(block.position.y);
      response.send(result);
      return;
    }

    iteration 

  }
}

