#pragma once

// ROS
#include "rclcpp/rclcpp.hpp"

// ROS Libraries
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

// Std Libraries
#include <cmath>
#include <vector>

class TurtleChaser : public rclcpp::Node
{
public:
    TurtleChaser();
private:
    // Ros variables
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr       p_sub_pose_1;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr       p_sub_pose_2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     p_pub_twist_2;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr            p_client_spawn;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr p_client_reset;

    // Callbacks
    void ChaseCallback(const turtlesim::msg::Pose::SharedPtr pose1);
    void PoseCallback(const turtlesim::msg::Pose::SharedPtr pose2);

    // Utils Functions
    void ResetTurtle2();
    bool SpawnTurtle2();

    // Server Response Handlers
    void SpawnHandleResponse(const rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future);
    void ResetHandleResponse(const rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture future);
    
    // Control Functions
    void CalculateDistance();
    void CalculateTarget(const turtlesim::msg::Pose::SharedPtr pose1);
    
    // Control Variables
    std::vector<double> target    = std::vector<double>(3, 0.0);
    std::vector<double> pose_self = std::vector<double>(3, 0.0);
    double distance               = std::numeric_limits<double>::max();
    double reset_threshold        = 0.50;
    double kp_linear              = 0.50;
    double kp_angular             = 0.80;
    double planning_horizon       = 0.25;
};