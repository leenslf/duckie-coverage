#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robot_msgs/msg/motor_speed_command.hpp>
#include "roboteq_driver/driver.h"

#define LEFT_MOTOR_ID 2
#define RIGHT_MOTOR_ID 1

#define GREEN_BUTTON_PIN_ID 7
#define BLUE_BUTTON_PIN_ID 6
#define RED_BUTTON_PIN_ID 4
#define SAFETY_BUTTON_PIN_ID 2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("roboteq_driver");
    Driver driver;

    robot_msgs::msg::MotorSpeedCommand cmd_msg;

    auto motor_command_sub = node->create_subscription<robot_msgs::msg::MotorSpeedCommand>(
        "/motor_commands", 1,
        [&cmd_msg](const robot_msgs::msg::MotorSpeedCommand::SharedPtr msg) {
            cmd_msg = *msg;
        });

    auto rpm_pub = node->create_publisher<robot_msgs::msg::MotorSpeedCommand>("/motor_rpms", 1);
    auto joy_status_pub = node->create_publisher<std_msgs::msg::Bool>("/joy_status", 10);

    int rpm_l, rpm_r;
    robot_msgs::msg::MotorSpeedCommand motor_rpms;
    rclcpp::Rate rate(100);

    while (rclcpp::ok())
    {
        rate.sleep();
        rclcpp::spin_some(node);

        if (driver.GetButtonStatus(SAFETY_BUTTON_PIN_ID))
        {
            driver.TurnWheelRPM(LEFT_MOTOR_ID, 0);
            driver.TurnWheelRPM(RIGHT_MOTOR_ID, 0);
        }
        else
        {
            driver.TurnWheelRPM(LEFT_MOTOR_ID, static_cast<int>(cmd_msg.left));
            driver.TurnWheelRPM(RIGHT_MOTOR_ID, static_cast<int>(cmd_msg.right));
        }

        if (!driver.GetMotorRPM(LEFT_MOTOR_ID, rpm_l) && !driver.GetMotorRPM(RIGHT_MOTOR_ID, rpm_r))
        {
            motor_rpms.header.stamp = node->now();
            motor_rpms.header.frame_id = "base_link";
            motor_rpms.left = rpm_l;
            motor_rpms.right = rpm_r;
            rpm_pub->publish(motor_rpms);
        }

        motor_rpms.left = 0;
        motor_rpms.right = 0;

        std_msgs::msg::Bool green_button_msg;
        green_button_msg.data = driver.GetButtonStatus(GREEN_BUTTON_PIN_ID);
        joy_status_pub->publish(green_button_msg);
    }

    rclcpp::shutdown();
    return 0;
}
