#include "../include/turtle_chaser/turtle_chaser.hpp"

TurtleChaser::TurtleChaser() : Node("turtle_chaser")
{
    this->p_client_spawn = this->create_client<turtlesim::srv::Spawn>("/spawn");
    this->SpawnTurtle2();
    
    this->p_sub_pose_1 = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 1, std::bind(&TurtleChaser::ChaseCallback, this, std::placeholders::_1));
    this->p_sub_pose_2 = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 1, std::bind(&TurtleChaser::PoseCallback, this, std::placeholders::_1));
    this->p_pub_twist_2 = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 1);
    this->p_client_reset = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle2/teleport_absolute");
}

void TurtleChaser::PoseCallback(const turtlesim::msg::Pose::SharedPtr pose2)
{
    this->pose_self[0] = pose2->x;
    this->pose_self[1] = pose2->y;
    this->pose_self[2] = pose2->theta;
    this->pose_self[3] = pose2->linear_velocity;
    this->pose_self[4] = pose2->angular_velocity;
}


void TurtleChaser::ChaseCallback(const turtlesim::msg::Pose::SharedPtr pose1)
{
    this->target[0] = pose1->x;
    this->target[1] = pose1->y;
    this->target[2] = pose1->theta;
    this->target[3] = pose1->linear_velocity;
    this->target[4] = pose1->angular_velocity;

    double distance = this->CalculateDistance();

    if(distance < reset_threshold)
        this->ResetTurtle2();

    auto control = geometry_msgs::msg::Twist();

    bool out_of_bounds = (this->pose_self[0] > 9 || this->pose_self[0] < 1) || (this->pose_self[1] > 9 || this->pose_self[1] < 1);

    if(/*this->first_out_of_bounds &&*/ out_of_bounds)
    {
        control.linear.x = 1;
        control.angular.z = 1;
    }
    else
    {
        this->first_out_of_bounds = false;
        control.linear.x = 3.0;

        double ang_error = std::atan2(this->target[1] - this->pose_self[1], this->target[0] - this->pose_self[0]) - this->pose_self[2];
        control.angular.z = 1.0 * ang_error;
    }
    this->p_pub_twist_2->publish(control);
}

bool TurtleChaser::SpawnTurtle2()
{
    RCLCPP_INFO(this->get_logger(), "Spawning Turtle2 ...");
    auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
    this->p_client_spawn->async_send_request(req, std::bind(&TurtleChaser::SpawnHandleResponse, this, std::placeholders::_1));
    return true;
}

void TurtleChaser::SpawnHandleResponse(const rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
{
    if(future.valid() && !future.get()->name.empty())
        RCLCPP_INFO(this->get_logger(), "Turtle %s was spawned correctly", future.get()->name.c_str());
    else
    {
        RCLCPP_ERROR(this->get_logger(), "FAILED TO SPAWN TURTLE 2");
        rclcpp::shutdown();
    }
}

double TurtleChaser::CalculateDistance()
{
    double distance = std::numeric_limits<double>::max();

    double dx = this->pose_self[0] - this->target[0];
    double dy = this->pose_self[1] - this->target[1];

    distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    
    return distance;
}

void TurtleChaser::ResetTurtle2()
{
    auto req = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    req->x = 5.5;
    req->y = 5.5;
    req->theta = 0.0;

    this->p_client_reset->async_send_request(req, std::bind(&TurtleChaser::ResetHandleResponse, this, std::placeholders::_1));
}

void TurtleChaser::ResetHandleResponse(const rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture future)
{
    if(future.valid() && !future.get()->structure_needs_at_least_one_member)
        RCLCPP_INFO(this->get_logger(), "Turtle2 reset successfully!");
    else
        RCLCPP_ERROR(this->get_logger(), "FAILED to reset turtle2");
}