#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>

class LidarNode : public rclcpp::Node
{
    public:
        LidarNode() : Node("lidar_node")
        {
            lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
            ("/scan", 10, std::bind(&LidarNode::scanCallback, this, std::placeholders::_1));
            // create_subscription<MessageType>(topic_name, queue_size, callback)
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

        
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            const float angle_min = msg->angle_min;
            const float angle_inc = msg->angle_increment;
            for(std::size_t i = 0; i < msg->ranges.size(); ++i){
                float range = msg->ranges[i];
                float theta, x, y;
                if(std::isfinite(range)){
                    theta = angle_min + i * angle_inc;
                    x = range * std::cos(theta);
                    y = range * std::sin(theta);
                    
                    RCLCPP_INFO(this->get_logger(),"Obstacle point: x=%.2f, y=%.2f",x,y);
                }
            }
        }

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}