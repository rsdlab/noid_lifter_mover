#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/srv/save_map.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("save_map_client_node");

  std::string map_topic = node->declare_parameter<std::string>("map_topic", "map");
  std::string map_url = node->declare_parameter<std::string>("map_url", "map");

  auto client = node->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");

  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Service not available.");
    return 1;
  }

  auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  request->map_topic = map_topic;
  request->map_url = map_url;

  auto result_future = client->async_send_request(request);

  // 結果をブロックして待つ
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Map saved successfully.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to save map.");
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

