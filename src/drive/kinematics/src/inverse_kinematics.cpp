#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robot_msgs/msg/motor_speed_command.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#define PI 3.14159265359

class InverseKinematicsNode : public rclcpp::Node
{
public:
    InverseKinematicsNode() : Node("inverse_kinematics_node"), p_x_(0.0), p_y_(0.0), p_th_(0.0)
    {
        declare_parameter("base_width", 0.1);
        declare_parameter("wheel_radius", 0.1);
        declare_parameter("reduction_rate", 1.0);

        base_width_     = get_parameter("base_width").as_double();
        wheel_radius_   = get_parameter("wheel_radius").as_double();
        reduction_rate_ = get_parameter("reduction_rate").as_double();

        rpm_sub_ = create_subscription<robot_msgs::msg::MotorSpeedCommand>(
            "/motor_rpms", 1,
            std::bind(&InverseKinematicsNode::rpmCallback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        last_time_ = now();
    }

private:
    void rpmCallback(const robot_msgs::msg::MotorSpeedCommand::SharedPtr msg)
    {
        double dt = (rclcpp::Time(msg->header.stamp) - last_time_).seconds();
        last_time_ = msg->header.stamp;

        double v_right = (2.0 * PI * wheel_radius_) * (msg->right / reduction_rate_) / 60.0;
        double v_left  = (2.0 * PI * wheel_radius_) * (msg->left  / reduction_rate_) / 60.0;

        double v_x  = (v_right + v_left) / 2.0;
        double v_th = (v_right - v_left) / base_width_;

        p_th_ += v_th * dt;
        p_x_  += v_x * std::cos(p_th_) * dt;
        p_y_  += v_x * std::sin(p_th_) * dt;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, p_th_);

        // Publish odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = msg->header.stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";
        odom.pose.pose.position.x  = p_x_;
        odom.pose.pose.position.y  = p_y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom_pub_->publish(odom);

        // Broadcast TF
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = msg->header.stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "base_link";
        tf.transform.translation.x = p_x_;
        tf.transform.translation.y = p_y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf);
    }

    double base_width_, wheel_radius_, reduction_rate_;
    double p_x_, p_y_, p_th_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<robot_msgs::msg::MotorSpeedCommand>::SharedPtr rpm_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
