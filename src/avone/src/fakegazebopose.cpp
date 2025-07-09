#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TeleportToOdomNode : public rclcpp::Node
{
public:
    TeleportToOdomNode() : Node("teleport_to_odom")
    {
        robot_name_ = "robot";
        latest_odom_ = nullptr;
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ackermann_steering_controller/odometry", 10,
            std::bind(&TeleportToOdomNode::odom_callback, this, std::placeholders::_1));
        // Timer at 10 Hz (set to faster/slower as you want)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&TeleportToOdomNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "TeleportToOdomNode started with 10Hz updates.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = msg;
    }

    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!latest_odom_) return;

        double x = latest_odom_->pose.pose.position.x;
        double y = latest_odom_->pose.pose.position.y;
        double z = latest_odom_->pose.pose.position.z;
        double qx = latest_odom_->pose.pose.orientation.x;
        double qy = latest_odom_->pose.pose.orientation.y;
        double qz = latest_odom_->pose.pose.orientation.z;
        double qw = latest_odom_->pose.pose.orientation.w;

        std::ostringstream cmd;
        cmd << "ign service -s /world/skidpad/set_pose "
            << "--reqtype ignition.msgs.Pose "
            << "--reptype ignition.msgs.Boolean "
            << "--timeout 1000 "
            << "--req 'name: \"" << robot_name_ << "\"\n"
            << "position: { x: " << x << ", y: " << y << ", z: " << z << " }\n"
            << "orientation: { x: " << qx << ", y: " << qy << ", z: " << qz << ", w: " << qw << " }'";

        // Print only every N calls (to avoid spam)
        static int print_counter = 0;
        if (++print_counter % 10 == 0)
            RCLCPP_INFO(this->get_logger(), "Running command:\n%s", cmd.str().c_str());

        int ret = std::system(cmd.str().c_str());
        if (ret != 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to call ign service (code %d)", ret);
        }
    }

    std::string robot_name_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    std::mutex odom_mutex_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleportToOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
