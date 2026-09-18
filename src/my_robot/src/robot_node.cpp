#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
//#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <chrono>
#include <iostream>

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("robot_node");

    //auto publisher = node -> create_publisher<std_msgs::msg::String>("robot_status",10);
    //auto publisher = node -> create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

    auto publisher = node -> create_publisher<sensor_msgs::msg::Imu>("imu/data",10);
    auto timer = node -> create_wall_timer(std::chrono::seconds(1),[publisher](){
            sensor_msgs::msg::Imu message;

            message.angular_velocity.x = 0.1;
            message.angular_velocity.y = 0.2;
            message.angular_velocity.z = 0.3;

            message.linear_acceleration.x = 1.0;
            message.linear_acceleration.y = 2.0;
            message.linear_acceleration.z = 9.81;

            publisher -> publish(message);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}