#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<rclcpp::Node>("imu_listener");

    auto subscription = node -> create_subscription<sensor_msgs::msg::Imu>("imu/data",10,[](const sensor_msgs::msg::Imu::SharedPtr message){
        std::cout << "Angular velocity:" << std::endl;
        std::cout << "x = "
                  << message->angular_velocity.x
                  << std::endl;
        std::cout << "y = "
                  << message->angular_velocity.y
                  << std::endl;
        std::cout << "z = "
                  << message->angular_velocity.z
                  << std::endl;

        std::cout << "Linear acceleration:" << std::endl;
        std::cout << "x = "
                  << message->linear_acceleration.x
                  << std::endl;
        std::cout << "y = "
                  << message->linear_acceleration.y
                  << std::endl;
        std::cout << "z = "
                  << message->linear_acceleration.z
                  << std::endl;
    });


    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}