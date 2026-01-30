#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cstdlib>
#include <string>
#include <sstream>

class BringupNavigationMonitor : public rclcpp::Node {
public:
    BringupNavigationMonitor()
    : Node("bringup_navigation_monitor"), triggered_(false)
    {
        this->declare_parameter<std::string>("robot_pkg_path", "");
        this->declare_parameter<bool>("simulation", true);
        this->declare_parameter<bool>("slam", false);
        this->declare_parameter<bool>("use_localization", true);
        this->declare_parameter<std::string>("map", "");

        robot_pkg_path_ = this->get_parameter("robot_pkg_path").as_string();
        simulation_ = this->get_parameter("simulation").as_bool();
        slam_mode_ = this->get_parameter("slam").as_bool();
        use_localization_ = this->get_parameter("use_localization").as_bool();
        map_ = this->get_parameter("map").as_string();

        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
             "/scan", 10, 
             std::bind(&BringupNavigationMonitor::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr) {
        if (!triggered_) {
            triggered_ = true;
            RCLCPP_INFO(this->get_logger(), "/scan topic received, launching navigation");

            sub_.reset();
  
            std::stringstream nav_cmd;
            if (simulation_) {
                nav_cmd << "ros2 launch " << robot_pkg_path_
                        << "/launch/parts/bringup_navigation.launch.py "
                        << "robot_pkg_path:=" << robot_pkg_path_ << " "
                        << "slam:=" << (slam_mode_ ? "True" : "False") << " "
                        << "use_sim_time:=True "
                        << "use_localization:=" << (use_localization_ ? "True" : "False") << " "
                        << "map:=" << map_ << " &";
            } else {
                nav_cmd << "ros2 launch " << robot_pkg_path_
                        << "/launch/parts/bringup_navigation.launch.py "
                        << "robot_pkg_path:=" << robot_pkg_path_ << " "
                        << "slam:=" << (slam_mode_ ? "True" : "False") << " "
                        << "use_sim_time:=False "
                        << "use_localization:=" << (use_localization_ ? "True" : "False") << " "
                        << "map:=" << map_ << " &";
            }
            std::system(nav_cmd.str().c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    std::string robot_pkg_path_;
    bool simulation_;
    bool slam_mode_;
    bool use_localization_;
    std::string map_;
    bool triggered_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BringupNavigationMonitor>());
    return 0;
}
