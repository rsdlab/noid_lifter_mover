#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <cstdlib>
#include <vector>
#include <string>

using ament_index_cpp::get_package_share_directory;

class TFStaticReadyMonitor : public rclcpp::Node
{
public:
  TFStaticReadyMonitor() : Node("tf_static_ready_monitor"), triggered_(false)
  {
    // パラメータ宣言
    this->declare_parameter<std::string>("cm_param_path", "");
    this->declare_parameter<std::string>("teleop_param_path", "");
    this->declare_parameter<std::string>("lidar_param_path", "");
    this->declare_parameter<bool>("simulation", true);

    cm_param_path_ = this->get_parameter("cm_param_path").as_string();
    teleop_param_path_ = this->get_parameter("teleop_param_path").as_string();
    lidar_param_path_ = this->get_parameter("lidar_param_path").as_string();
    simulation_ = this->get_parameter("simulation").as_bool();

    // QoS設定 + サブスクライブ
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/tf_static_ready", qos,
      std::bind(&TFStaticReadyMonitor::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !triggered_) {
      triggered_ = true;
      RCLCPP_INFO(this->get_logger(), "/tf_static_ready received – launching controller_manager");

      launch_process({
        "ros2", "run",
        "controller_manager", "ros2_control_node",
        "--ros-args",
        "--params-file", cm_param_path_,
        "--remap", "~/robot_description:=/robot_description",
        "--remap", "/mechanum_controller/cmd_vel_nav:=/cmd_vel_nav"
      }, "controller_manager");

      launch_process({
        "ros2", "run",
        "joy_linux", "joy_linux_node",
      }, "joy_linux");

      launch_process({
        "ros2", "run",
        "teleop_twist_joy", "teleop_node",
        "--ros-args",
        "--params-file", teleop_param_path_,
        "--remap", "/cmd_vel:=/mechanum_controller/cmd_vel_teleop",
      }, "teleop_twist_joy_node");
      
      if (!simulation_) {
        launch_process({
          "ros2", "launch",
          "urg_node2", "urg_node2.launch.py",
          "scan_topic_name:=scan_raw"
        }, "urg_node2");

        launch_process({
          "ros2", "run",
          "laser_filters", "scan_to_scan_filter_chain",
          "--ros-args",
          "--params-file", lidar_param_path_,
          "--remap", "/scan:=/scan_raw",
          "--remap", "/scan_filtered:=/scan"
        }, "laser_filter");
      }

    }
  }

  void launch_process(const std::vector<std::string>& args_str, const std::string& label)
  {
    std::vector<char*> args;
    for (const auto& s : args_str) {
      args.push_back(const_cast<char*>(s.c_str()));
    }
    args.push_back(nullptr);

    pid_t pid = fork();
    if (pid == 0) {
      execvp(args[0], args.data());
      std::perror(("execvp failed for " + label).c_str());
      std::exit(1);
    } else if (pid > 0) {
      RCLCPP_INFO(this->get_logger(), "%s launched with PID %d", label.c_str(), pid);
    } else {
      RCLCPP_ERROR(this->get_logger(), "fork failed for %s", label.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  std::string cm_param_path_;
  std::string teleop_param_path_;
  std::string lidar_param_path_;
  bool simulation_;
  bool triggered_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFStaticReadyMonitor>());
  return 0;
}
