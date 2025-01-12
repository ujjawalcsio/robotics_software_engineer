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
        // Declare parameters with default values
        this->declare_parameter<int>("mode", 1);
        this->declare_parameter<double>("radius", 1.0);

        // Get initial parameters
        this->get_parameter("mode", mode);
        this->get_parameter("radius", radius);

        // Parameter callback for runtime updates
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MinimalPublisher::parameter_callback, this, std::placeholders::_1));

        // Validate parameters
        if (mode == 1 && radius <= 0.0)
        {
            RCLCPP_ERROR(this->get_logger(), "Radius must be positive for circular movement. Setting to default 1.0.");
            radius = 1.0;
        }

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&MinimalPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "MinimalPublisher node initialized with mode: %d, radius: %.2f", mode, radius);
    }

private:
    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param : parameters)
        {
            if (param.get_name() == "mode" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                mode = param.as_int();
                RCLCPP_INFO(this->get_logger(), "Updated mode: %d", mode);
            }
            else if (param.get_name() == "radius" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                radius = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated radius: %.2f", radius);
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Parameters updated successfully.";
        return result;
    }

    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        if (mode == 1)
        {
            // Circular movement
            double linear_velocity = 1.0; // Adjust as needed
            double angular_velocity = linear_velocity / radius; // ω = v / r

            message.linear.x = linear_velocity;
            message.angular.z = angular_velocity;

            RCLCPP_INFO(this->get_logger(), "Circular Movement - Linear.x: %.2f, Angular.z: %.2f", linear_velocity, angular_velocity);
        }
        else if (mode == 2)
        {
            // Spiral movement
            current_radius += 0.01;        // Gradually increase radius
            double linear_velocity = current_radius; // Linear velocity proportional to current radius
            double angular_velocity = 1;           // Fixed angular velocity

            message.linear.x = linear_velocity;
            message.angular.z = angular_velocity;

            RCLCPP_INFO(this->get_logger(), "Spiral Movement - Linear.x: %.2f, Angular.z: %.2f", linear_velocity, angular_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown mode: %d. No movement command sent.", mode);
        }

        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    int mode;
    double radius;
    double current_radius;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
