#include "../include/turtle_chaser/turtle_chaser.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleChaser>());
    rclcpp::shutdown();
    
    return 0;
}