#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/bool.hpp"
#include <thread>
#include <chrono>

class TFStaticMonitor : public rclcpp::Node
{
public:
  TFStaticMonitor()
  : Node("tf_static_monitor")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", qos,
      std::bind(&TFStaticMonitor::callback_tf_static, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<std_msgs::msg::Bool>("tf_static_ready", qos);

    RCLCPP_INFO(this->get_logger(), "Waiting for /tf_static...");
  }

private:
  void callback_tf_static(const tf2_msgs::msg::TFMessage::SharedPtr /*msg*/)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = true;
    pub_->publish(ready_msg);

    RCLCPP_INFO(this->get_logger(), "/tf_static detected → /tf_static_ready published");

    // 終了処理
    rclcpp::shutdown();
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFStaticMonitor>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  return 0;
}

