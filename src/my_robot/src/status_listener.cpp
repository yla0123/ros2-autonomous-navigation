#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("status_listener");
    // auto subscription = node -> create_subscription<std_msgs::msg::String>("robot_status", 10, 
    //     [](const std_msgs::msg::String::SharedPtr message)
    //     {
    //         std::cout << "Motor command received: " << message->data << std::endl;
    //     });

    auto publisher = node -> create_publisher<std_msgs::msg::String>("robot_response",10);
    std_msgs::msg::String message_p;
    message_p.data =  "Twist received";

    auto subscription = node -> create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,[publisher,message_p](geometry_msgs::msg::Twist::SharedPtr message){
        std::cout<<"Motor velocity x:"<<message->linear.x<<std::endl;
        std::cout<<"Motor angular velocity y:"<<message->angular.y<<std::endl;     
        
        publisher->publish(message_p);
    });



    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}