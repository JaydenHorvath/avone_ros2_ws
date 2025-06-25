#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TwistToTwistStamped : public rclcpp::Node
{
public:
    TwistToTwistStamped()
    : Node("twist_to_twiststamped")
    {
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&TwistToTwistStamped::twist_callback, this, std::placeholders::_1)
        );
        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/ackermann_steering_controller/reference", 10
        );
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto stamped = geometry_msgs::msg::TwistStamped();
        stamped.header.stamp = now();
        stamped.header.frame_id = ""; // or "base_link" if you prefer
        stamped.twist = *msg;
        pub_->publish(stamped);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToTwistStamped>());
    rclcpp::shutdown();
    return 0;
}
