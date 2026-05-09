#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot_msgs/msg/motor_speed_command.hpp>

#define PI 3.14159265359

class KinematicsNode : public rclcpp::Node
{
public:
    KinematicsNode() : Node("kinematics_node")
    {
        declare_parameter("base_width", 0.1);
        declare_parameter("wheel_radius", 0.1);
        declare_parameter("reduction_rate", 1.0);

        base_width_ = get_parameter("base_width").as_double();
        wheel_radius_ = get_parameter("wheel_radius").as_double();
        reduction_rate_ = get_parameter("reduction_rate").as_double();

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 1,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmd_vel_ = *msg;
            });

        motor_cmd_pub_ = create_publisher<robot_msgs::msg::MotorSpeedCommand>("motor_commands", 1);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&KinematicsNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        double vr = cmd_vel_.linear.x + cmd_vel_.angular.z * base_width_ / 2.0;
        double vl = cmd_vel_.linear.x - cmd_vel_.angular.z * base_width_ / 2.0;

        robot_msgs::msg::MotorSpeedCommand msg;
        msg.right = (60.0 / (2.0 * PI * wheel_radius_)) * vr * reduction_rate_;
        msg.left  = (60.0 / (2.0 * PI * wheel_radius_)) * vl * reduction_rate_;

        motor_cmd_pub_->publish(msg);
    }

    double base_width_, wheel_radius_, reduction_rate_;
    geometry_msgs::msg::Twist cmd_vel_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<robot_msgs::msg::MotorSpeedCommand>::SharedPtr motor_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
