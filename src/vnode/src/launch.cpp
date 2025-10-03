// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/wait_for_message.hpp>

// std Headers
#include <chrono>

// Services
#include "mineros_inter/srv/bot_pos.hpp"
#include "mineros_inter/srv/move_to.hpp"
#include "mineros_inter/srv/find_blocks.hpp"

// Messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#define MINEROS_SETPOS_TOPIC "/mineros/set_position"
#define MINEROS_GETPOS_TOPIC "/mineros/local_position/pose"
#define MINEROS_GETBLOCKS_TOPIC "/mineros/mining/find_blocks"

using std::placeholders::_1;
using std::placeholders::_2;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseArray;
using mineros_inter::srv::MoveTo;
using namespace std::chrono_literals;

class MinerosNode : public rclcpp::Node {
		rclcpp::Subscription<PoseStamped>::SharedPtr pose_subscriber_;
		rclcpp::Client<MoveTo>::SharedPtr moveto_client_;
    public:
        MinerosNode() : Node("mineros_node") {
					// Add build date for sanity checks.
					RCLCPP_INFO(this->get_logger(), "Running vnode built on %s at %s", __DATE__, __TIME__);

					// Set QoS to whatever /mineros/local_position/pose uses
					rclcpp::QoS qos(rclcpp::KeepLast(10));
						qos.reliable();
						qos.transient_local();

					// Subscriber: Get bot position.
					pose_subscriber_ = this->create_subscription<PoseStamped>(
						MINEROS_GETPOS_TOPIC, 
						qos,
						std::bind(&MinerosNode::pose_callback, this, _1));
          RCLCPP_INFO(this->get_logger(), "[+] Subscribed to %s", MINEROS_GETPOS_TOPIC);

					// Client: Set bot position.
					moveto_client_ = this->create_client<MoveTo>(MINEROS_SETPOS_TOPIC);

					while (!moveto_client_->wait_for_service(1s)) {
						RCLCPP_INFO(this->get_logger(), "Waiting for MoveTo service...");
					}
        }

			private:
				PoseStamped pose;
				PoseStamped goal_pose;
				bool moveto_success = true;
				void pose_callback(const PoseStamped::SharedPtr msg) {
					RCLCPP_INFO(this->get_logger(), "Received pose: [%.2f,%.2f,%.2f]",
							msg->pose.position.x,
							msg->pose.position.y,
							msg->pose.position.z);
					this->pose = *msg;

					// Make request to MoveTo service
					auto request = std::make_shared<MoveTo::Request>();
					request->pose.header.frame_id = "map";
					request->pose.pose.position.y = 108.0;
					this->goal_pose.pose.position.y = 108.0;

					// Only set new goal if goal has been reached.
					if (this->moveto_success) {
						this->goal_pose.pose.position.x = this->pose.pose.position.x;
						this->goal_pose.pose.position.z = this->pose.pose.position.z + 5.00;
						request->pose.pose.position.x = this->goal_pose.pose.position.x;
						request->pose.pose.position.z = this->goal_pose.pose.position.z;
					}
					else {
						request->pose.pose.position.x = this->goal_pose.pose.position.x;
						request->pose.pose.position.z = this->goal_pose.pose.position.z;
					}

					// Set goal position to end of parkour
					//
					if (false) {
						request->pose.pose.position.x = 108.0;
						request->pose.pose.position.y = 102.0;
						request->pose.pose.position.z = 123.0;
					}

					RCLCPP_INFO(this->get_logger(), "Sending goal pose: [%.2f,%.2f,%.2f]",
							this->goal_pose.pose.position.x,
							this->goal_pose.pose.position.y,
							this->goal_pose.pose.position.z);
					auto future = this->moveto_client_->async_send_request(request, [this](rclcpp::Client<MoveTo>::SharedFuture result) {
						if (result.get()->success) {
							this->moveto_success = true;
						}
						else {
							this->moveto_success = false;
						}
					});
				}
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
		auto main_node = std::make_shared<MinerosNode>();
    rclcpp::spin(main_node);
    rclcpp::shutdown();
    return 0;
}
