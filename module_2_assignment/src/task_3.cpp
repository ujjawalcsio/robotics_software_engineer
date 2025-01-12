#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TurtleController : public rclcpp::Node
{
public:
    TurtleController()
        : Node("turtle_controller"), forward_(true)
    {
        this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");

        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        timer_ = this->create_wall_timer(
            2s, std::bind(&TurtleController::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        if (forward_)
        {
            message.linear.x = 1.0;  // Move forward
        }
        else
        {
            message.linear.x = -1.0; // Move backward
        }
        forward_ = !forward_; // Toggle direction
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Moving %s",
                    forward_ ? "forward" : "backward");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    bool forward_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleController>());
    rclcpp::shutdown();
    return 0;
}
