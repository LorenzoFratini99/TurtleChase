#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller")
    {
        this->pose_sub    = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 1, std::bind(&TurtleController::ControlCallback, this, std::placeholders::_1));
        this->command_pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 1);
    }

private:

    // Callbacks
    void ControlCallback(const turtlesim::msg::Pose::SharedPtr msg);

    // Private ROS variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr   pose_sub;
};