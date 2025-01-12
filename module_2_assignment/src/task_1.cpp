#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher"), mode(1), radius(1.0), current_radius(0.5)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // Get user input for mode and radius
        RCLCPP_INFO(this->get_logger(), "Enter movement mode (1 for circle, 2 for spiral): ");
        std::cin >> mode;

        if (mode == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Enter radius for the circle: ");
            std::cin >> radius;
            if (radius <= 0.0)
            {
                RCLCPP_ERROR(this->get_logger(), "Radius must be positive. Setting default radius to 1.0.");
                radius = 1.0;
            }
        }

        timer_ = this->create_wall_timer(
            100ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        if (mode == 1)
        {
            // Circular movement
            double linear_velocity = 1.0; // Adjust as needed
            message.linear.x = linear_velocity;
            message.angular.z = linear_velocity / radius; // Angular velocity based on radius
        }
        else if (mode == 2)
        {
            // Spiral movement
            current_radius += 0.01;        // Gradually increase radius
            message.linear.x = current_radius; // Linear velocity proportional to current radius
            message.angular.z = 1.0;           // Fixed angular velocity
        }
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;    // Timer for periodic callbacks
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // Publisher for Twist messages
    int mode;                               // Movement mode (1 for circle, 2 for spiral)
    double radius;                          // Radius for circular movement
    double current_radius;                  // Current radius for spiral movement
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
