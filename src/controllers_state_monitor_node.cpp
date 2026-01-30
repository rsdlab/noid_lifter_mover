#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "std_msgs/msg/bool.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include <string>
#include <sstream>
#include <vector>
#include <map>

class ControllersStateMonitor : public rclcpp::Node {
public:
  ControllersStateMonitor(const std::vector<std::string> & target_nodes)
  : Node("controller_state_monitor"), launched_(false)
  {
    this->declare_parameter<std::string>("robot_pkg_path", "");
    this->declare_parameter<std::string>("dummy_map", "");
    
    robot_pkg_path_ = this->get_parameter("robot_pkg_path").as_string();
    dummy_map_ = this->get_parameter("dummy_map").as_string();
  
    for (const auto & name : target_nodes) {
      states_[name] = "unknown";

      auto sub = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        "/" + name + "/transition_event", 10,
        [this, name](lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
          if (!rclcpp::ok()) {
            return;
          }

          if (msg->goal_state.label == "active") {
            RCLCPP_INFO(get_logger(), "%s is now ACTIVE", name.c_str());
            states_[name] = "active";
            check_all_active();
          } else {
            RCLCPP_INFO(get_logger(), "%s transitioned to %s",
              name.c_str(), msg->goal_state.label.c_str());
          }
        });
      subs_.push_back(sub);
    }
  }

private:
  bool all_active(const std::string &label,
                  const std::vector<std::string> &targets) {
    bool ok = true;
    for (const auto & name : targets) {
      auto it = states_.find(name);
      if (it == states_.end()) {
        RCLCPP_WARN(this->get_logger(),
                    "[%s not register] %s's state does not exist in states_",
                    label.c_str(), name.c_str());
        ok = false;
      } else if (it->second != "active") {
        RCLCPP_INFO(this->get_logger(),
                    "[%s not completed] %s state=%s",
                    label.c_str(), name.c_str(), it->second.c_str());
        ok = false;
      }
    }
    return ok;
  }

  void check_all_active() {
    if (!rclcpp::ok()) return;
    if (launched_) return;

    std::vector<std::string> targets;
    for (const auto & pair : states_) {
      targets.push_back(pair.first);
    }

    if (!all_active("dummy_lidar", targets)) {
      return; 
    }

    RCLCPP_INFO(get_logger(), "All nodes are ACTIVE. Launching Dummy_lidar...");
    
    launched_ = true;
    subs_.clear(); 
    
    std::stringstream nav_cmd;
    nav_cmd << "ros2 launch " << robot_pkg_path_
            << "/launch/parts/bringup_dummy_lidar.launch.py "
            << "dummy_map:=" << dummy_map_ << " &";
    std::system(nav_cmd.str().c_str());

    rclcpp::shutdown();
  }

  std::vector<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr> subs_;
  std::map<std::string, std::string> states_;
  bool launched_;
  std::string robot_pkg_path_;
  std::string dummy_map_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  std::vector<std::string> names = {
    "mechanum_controller", 
    "joint_state_broadcaster"
  };

  auto node = std::make_shared<ControllersStateMonitor>(names);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
