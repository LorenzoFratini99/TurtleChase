#include "../include/turtle_controller/turtle_controller.hpp"

TurtleController::TurtleController() : Node("turtle_controller")
{
  // Subscribers
  this->pose_sub = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 1, std::bind(&TurtleController::ControlCallback, this, std::placeholders::_1));
  
  // Publishers
  this->command_pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 1);
}

void TurtleController::ControlCallback(turtlesim::msg::Pose::SharedPtr msg)
{
  geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();

  // Gathering Current Pose
  double x = msg->x;
  double y = msg->y;
  double theta;
  if(msg->theta < 0.0)
    theta = 2.0 * M_PI - std::abs(msg->theta);
  else
    theta = msg->theta;

  // Out of Bound calculation
  bool out_of_bounds = (x > __OUT_OF_BOUNDS_P_H_LIMIT__ || x < __OUT_OF_BOUNDS_P_L_LIMIT__) || 
                       (y > __OUT_OF_BOUNDS_P_H_LIMIT__ || y < __OUT_OF_BOUNDS_P_L_LIMIT__);
  
  // Control Action calculation
  if(out_of_bounds && !this->was_out_of_bounds)
  {
    this->was_out_of_bounds = true;
    this->theta_in = theta;
    this->rotation_direction = this->ComputeRotationDirection(theta, x, y);
    twist.linear.x  = 1.0;
    twist.angular.z = this->rotation_direction * 1.0;
    // RCLCPP_INFO(this->get_logger(), "1, %d, %d, %d", out_of_bounds, this->was_out_of_bounds, this->rotation_direction); // DEBUG
  }
  else if(out_of_bounds && this->was_out_of_bounds && std::abs(this->theta_in - theta) < __OUT_OF_BOUNDS_ROT_LIMIT__)
  {
    twist.linear.x  = 1.0;
    twist.angular.z = this->rotation_direction * 1.0;
    // RCLCPP_INFO(this->get_logger(), "2, %d, %d, %d", out_of_bounds, this->was_out_of_bounds, this->rotation_direction); // DEBUG
  }
  else if(out_of_bounds && this->was_out_of_bounds && std::abs(this->theta_in - theta) > __OUT_OF_BOUNDS_ROT_LIMIT__)
  {
    twist.linear.x  = 1.0;
    twist.angular.z = this->rotation_direction * 0.75;
    // RCLCPP_INFO(this->get_logger(), "3, %d, %d, %d", out_of_bounds, this->was_out_of_bounds, this->rotation_direction); // DEBUG
  }
  else
  {
    twist.linear.x = 2.0;
    this->was_out_of_bounds = false;
    // RCLCPP_INFO(this->get_logger(), "4, %d, %d, %d", out_of_bounds, was_out_of_bounds, rotation_direction); // DEBUG
  }

  this->command_pub->publish(twist);
}

int TurtleController::ComputeRotationDirection(const double& theta, const double& x, const double& y)
{
  bool out_N = false;
  bool out_S = false;
  bool out_E = false;
  bool out_W = false;
  // bool out_NW = false;
  // bool out_NE = false;
  // bool out_SE = false;
  // bool out_SW = false;

  // Where am I?
  if(x > __OUT_OF_BOUNDS_P_H_LIMIT__)
    out_E = true;
  
  if(x < __OUT_OF_BOUNDS_P_L_LIMIT__)
    out_W = true;

  if(y > __OUT_OF_BOUNDS_P_H_LIMIT__)
    out_N = true;

  if(y < __OUT_OF_BOUNDS_P_L_LIMIT__)
    out_S = true;

  // DEBUG
  // RCLCPP_INFO(this->get_logger(), "%d, %d, %d, %d,", out_N, out_E, out_S, out_W);
  // RCLCPP_INFO(this->get_logger(), "%f", theta);

  // TODO
  // out_NW = out_N && out_W;
  // out_SW = out_S && out_W;
  // out_NE = out_N && out_E;
  // out_SE = out_S && out_E;

  // Rotation Direction Computation
  if(out_N && theta > M_PI_2)
    return 1;
  else if(out_N && theta < M_PI_2)
    return -1;

  if(out_S && theta > 3.0 * M_PI / 2.0)
    return 1;
  else if(out_S && theta < 3.0 * M_PI / 2.0)
    return -1;

  if(out_W && theta < M_PI)
    return -1;
  else if(out_W && theta > M_PI)
    return 1;

  if(out_E && theta < M_PI)
    return 1;
  else if(out_E && theta > M_PI)
    return -1;

  return 1;
}