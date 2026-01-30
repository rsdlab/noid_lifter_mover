#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include <thread>
#include <chrono>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

class BringupRviz2Monitor : public rclcpp::Node
{
public:
  BringupRviz2Monitor()
  : Node("bringup_rviz2_monitor")
  {
    // MoveGroup アクションサーバーを監視
    client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
      this, "/move_action");

    // タイマーで定期的にサーバーの存在をチェック
    timer_ = this->create_wall_timer(500ms, std::bind(&BringupRviz2Monitor::check_server, this));
  }

private:
  void check_server()
  {
    if (client_->wait_for_action_server(0ms)) {
      RCLCPP_INFO(this->get_logger(), "MoveGroup Action Server is ready!");
      timer_->cancel();
      rclcpp::shutdown();  
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Waiting for MoveGroup Action Server...");
    }
  }

  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BringupRviz2Monitor>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  return 0;
}

