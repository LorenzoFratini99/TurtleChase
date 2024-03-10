#include "../include/turtle_controller/turtle_controller.hpp"


void TurtleController::ControlCallback(turtlesim::msg::Pose::SharedPtr msg)
{
  auto twist = geometry_msgs::msg::Twist();

  double x = msg->x;
  double y = msg->y;

  bool out_of_bounds = (x > 9 || x < 1) || (y > 9 || y < 1);
  
  if(out_of_bounds)
  {
    twist.linear.x = 1;
    twist.angular.z = 1;
  }
  else
    twist.linear.x = 3;
  
  this->command_pub->publish(twist);
}