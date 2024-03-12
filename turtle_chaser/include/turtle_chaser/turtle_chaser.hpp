#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <cmath>

#include <vector>

class TurtleChaser : public rclcpp::Node
{
public:
    TurtleChaser();
private:
    // Ros shared ptrs
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr p_sub_pose_1;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr p_sub_pose_2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr p_pub_twist_2;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr p_client_spawn;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr p_client_reset;

    // callbacks
    void ChaseCallback(const turtlesim::msg::Pose::SharedPtr pose1);
    void PoseCallback(const turtlesim::msg::Pose::SharedPtr pose2);
    void SpawnHandleResponse(const rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future);
    void ResetHandleResponse(const rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture future);
    // functions
    void ResetTurtle2();
    bool SpawnTurtle2();
    double CalculateDistance(); // include also the heading in the future

    // control variables
    std::vector<double> target = std::vector<double>(5, 0.0);
    std::vector<double> pose_self = std::vector<double>(5, 0.0);
    double reset_threshold = 0.5;
    bool first_out_of_bounds = true;
};