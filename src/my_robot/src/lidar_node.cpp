#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>
#include <vector>
#include <utility>

class LidarNode : public rclcpp::Node
{
    public:
        LidarNode() : Node("lidar_node")
        {
            lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
            ("/scan", 10, std::bind(&LidarNode::scanCallback, this, std::placeholders::_1));
            // create_subscription<MessageType>(topic_name, queue_size, callback)

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&LidarNode::odomCallback, this, std::placeholders::_1));
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        double robot_x_ = 0.0;
        double robot_y_ = 0.0;
        double robot_theta_ = 0.0;

        const double lidar_offset_x = 0.2;
        const double lidar_offset_y = 0.0;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            robot_x_ = msg->pose.pose.position.x;
            robot_y_ = msg->pose.pose.position.y;

            double z = msg->pose.pose.orientation.z;
            double w = msg->pose.pose.orientation.w;

            robot_theta_ = 2.0 * std::atan2(z, w);
            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "Robot pose: x=%.2f, y=%.2f, theta=%.2f",
            //     robot_x_,
            //     robot_y_,
            //     robot_theta_
            // );
        }

        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            const float angle_min = msg->angle_min;
            const float angle_inc = msg->angle_increment;
            std::size_t printed = 0;

            std::vector<std::pair<float, float>> obstacle;

            for(std::size_t i = 0; i < msg->ranges.size(); ++i){
                const float range = msg->ranges[i];

                if(! std::isfinite(range)){
                    continue;
                }
                if(range < msg->range_min || range > msg->range_max){
                    continue;
                }
                
                const float theta = angle_min + i * angle_inc;
                const float x = range * std::cos(theta) + lidar_offset_x;
                const float y = range * std::sin(theta) + lidar_offset_y;
                // if(printed < 3){
                //     RCLCPP_INFO(this->get_logger(),"Obstacle point: x=%.2f, y=%.2f",x,y);
                //     ++printed;
                // }

                const float x_odom = robot_x_ + x*std::cos(robot_theta_) - y*std::sin(robot_theta_);
                const float y_odom = robot_y_ + x*std::sin(robot_theta_) + y*std::cos(robot_theta_);

                obstacle.emplace_back(x_odom, y_odom);
            }

            // RCLCPP_INFO(this->get_logger(), "detected: %zu points", obstacle.size());
            // %zu for size_t

            if (!obstacle.empty()){
                RCLCPP_INFO(this->get_logger(), "First odom obstacle: x=%.2f, y=%.2f", obstacle[0].first, obstacle[0].second);
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