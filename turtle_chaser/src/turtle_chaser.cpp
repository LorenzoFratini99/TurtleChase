#include "../include/turtle_chaser/turtle_chaser.hpp"

TurtleChaser::TurtleChaser() : Node("turtle_chaser")
{
    // Clients
    this->p_client_spawn = this->create_client<turtlesim::srv::Spawn>("/spawn");
    this->p_client_reset = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle2/teleport_absolute");

    // Subscribers
    this->p_sub_pose_1 = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 1, std::bind(&TurtleChaser::ChaseCallback, this, std::placeholders::_1));
    this->p_sub_pose_2 = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 1, std::bind(&TurtleChaser::PoseCallback, this, std::placeholders::_1));
    
    // Publishers
    this->p_pub_twist_2 = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 1);

    // Spawining the second turtle
    this->SpawnTurtle2();

    RCLCPP_INFO(this->get_logger(), "Turtle Chaser setup finished!");
}

// Callbacks
void TurtleChaser::PoseCallback(const turtlesim::msg::Pose::SharedPtr pose2)
{
    this->pose_self[0] = pose2->x;
    this->pose_self[1] = pose2->y;
    
    if(pose2->theta < 0.0)
        this->pose_self[2] = 2.0 * M_PI - std::abs(pose2->theta);
    else
        this->pose_self[2] = pose2->theta;
}

void TurtleChaser::ChaseCallback(const turtlesim::msg::Pose::SharedPtr pose1)
{
    // Computing Target Pose and Distance to Target
    this->CalculateTarget(pose1);
    this->CalculateDistance();

    // Goal Check
    if(this->distance < this->reset_threshold)
        this->ResetTurtle2();

    // Control Action Computation
    geometry_msgs::msg::Twist control = geometry_msgs::msg::Twist();

    // Linear Contribution
    double u_linear = this->kp_linear * this->distance;
    control.linear.x = (u_linear > 4.0) ? 4.0 : u_linear;

    // Angular Contribution
    double target_heading = std::atan2(this->target[1] - this->pose_self[1], this->target[0] - this->pose_self[0]); // angle between target and this
    target_heading = (target_heading < 0.0) ? 2.0 * M_PI - std::abs(target_heading) : target_heading; // same angle, but in 360°
    double heading_error = target_heading - this->pose_self[2];
    int sign = (heading_error > 0.0) ? 1 : -1;

    heading_error = (std::abs(heading_error) > M_PI) ? (-sign*(2.0 * M_PI - std::abs(heading_error))) : heading_error; // choosing the shortest rotation

    control.angular.z = this->kp_angular * heading_error;

    this->p_pub_twist_2->publish(control);
}

// Utils Functions
bool TurtleChaser::SpawnTurtle2()
{
    RCLCPP_INFO(this->get_logger(), "Spawning Turtle2 ...");
    std::shared_ptr<turtlesim::srv::Spawn::Request> req = std::make_shared<turtlesim::srv::Spawn::Request>();
    this->p_client_spawn->async_send_request(req, std::bind(&TurtleChaser::SpawnHandleResponse, this, std::placeholders::_1));
    return true;
}

void TurtleChaser::ResetTurtle2()
{
    auto req = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    req->x = 5.5;
    req->y = 5.5;
    req->theta = 0.0;

    this->p_client_reset->async_send_request(req, std::bind(&TurtleChaser::ResetHandleResponse, this, std::placeholders::_1));
}

// Server Response Handlers
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

void TurtleChaser::ResetHandleResponse(const rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture future)
{
    if(future.valid() && !future.get()->structure_needs_at_least_one_member)
        RCLCPP_INFO(this->get_logger(), "Turtle2 reset successfully!");
    else
        RCLCPP_ERROR(this->get_logger(), "FAILED to reset turtle2");
}

// Control Functions
void TurtleChaser::CalculateDistance()
{
    double dx = this->pose_self[0] - this->target[0];
    double dy = this->pose_self[1] - this->target[1];

    this->distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}

void TurtleChaser::CalculateTarget(const turtlesim::msg::Pose::SharedPtr pose1)
{
    this->target[0] = pose1->x;
    this->target[1] = pose1->y;

    if(pose1->theta < 0.0)
        this->target[2] = 2.0 * M_PI - std::abs(pose1->theta);
    else
        this->target[2] = pose1->theta;

    this->target[3] = pose1->linear_velocity;
    this->target[4] = pose1->angular_velocity;

    double dx = 0.1;
    this->target[0] += pose1->linear_velocity * this->planning_horizon * cos(this->target[2]);
    this->target[1] += pose1->linear_velocity * this->planning_horizon * sin(this->target[2]);

    this->target[0] += pose1->angular_velocity * this->planning_horizon * dx * cos(this->target[2] + M_PI_2);
    this->target[1] += pose1->angular_velocity * this->planning_horizon * dx * sin(this->target[2] + M_PI_2);
}