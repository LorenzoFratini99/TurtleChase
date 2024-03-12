#pragma once

// ROS
#include "rclcpp/rclcpp.hpp"

// ROS Libraires
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

// Scene Boundaries
#define __OUT_OF_BOUNDS_ROT_LIMIT__ M_PI_2 - 0.25
#define __OUT_OF_BOUNDS_P_H_LIMIT__ 9.0
#define __OUT_OF_BOUNDS_P_L_LIMIT__ 1.0

class TurtleController : public rclcpp::Node
{
public:
    TurtleController();

private:
    // Callbacks
    void ControlCallback(const turtlesim::msg::Pose::SharedPtr msg);

    // Private ROS variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr   pose_sub;

    // Control Variables
    double theta_in           = 0.0;
    bool   was_out_of_bounds  = false;
    int    rotation_direction = 0;
    
    // Control Functions
    int ComputeRotationDirection(const double& theta, const double& x, const double& y);
};