#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using namespace std::chrono_literals;

class AmclStateMonitor : public rclcpp::Node
{
public:
  AmclStateMonitor()
    : Node("amcl_state_monitor"), triggered_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Amcl state monitor started");

    state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/amcl/get_state");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&AmclStateMonitor::poll_amcl_state, this));
  }

private:
  void poll_amcl_state()
  {
    if (!state_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Service /amcl/get_state not yet available");
      return;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    auto future = state_client_->async_send_request(request,
      [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future_response) {
        auto response = future_response.get();
        if (response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          if (!triggered_) {
            triggered_ = true;
            RCLCPP_INFO(this->get_logger(), "AMCL is active. Initial pose should be published externally.");
          }
        } else {
          RCLCPP_INFO(this->get_logger(), "AMCL current state: %s", response->current_state.label.c_str());
        }
      });
  }

  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr state_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool triggered_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AmclStateMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

