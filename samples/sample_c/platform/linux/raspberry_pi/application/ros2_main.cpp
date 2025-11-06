/**
 * @file ros2_main.cpp
 * @brief ROS2 initialization wrapper for DJI Payload SDK application
 */

#include <rclcpp/rclcpp.hpp>

// C wrapper function to initialize ROS2
extern "C" void ros2_init_wrapper(int argc, char **argv)
{
    rclcpp::init(argc, argv);
}
