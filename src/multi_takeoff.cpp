#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdlib>

void takeoff_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::cout << "Takeoff command received: " << msg->data << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::Node::SharedPtr takeoff_node = rclcpp::Node::make_shared("takeoff_node");
    auto takeoff_sub = takeoff_node->create_subscription<std_msgs::msg::String>(
        "takeoff", 10, takeoff_callback);

    rclcpp::spin(takeoff_node);
    rclcpp::shutdown();
    return 0;
}



