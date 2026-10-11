#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <cmath>
#include <vector>
#include <utility>

class LidarNode : public rclcpp::Node
{
    public:
        LidarNode() : Node("lidar_node")
        {
            grid_.assign(width_ * height_, -1);

            lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
            ("/scan", 10, std::bind(&LidarNode::scanCallback, this, std::placeholders::_1));
            // create_subscription<MessageType>(topic_name, queue_size, callback)

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&LidarNode::odomCallback, this, std::placeholders::_1));

            map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

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
            // std::size_t printed = 0;

            // std::vector<std::pair<float, float>> obstacle;
            // std::vector<std::pair<int,int>> obstacle_cells;
            
            for(std::size_t i = 0; i < msg->ranges.size(); ++i){
                const float range = msg->ranges[i];

                if(! std::isfinite(range)){
                    continue;
                }
                if(range < msg->range_min || range > msg->range_max){
                    continue;
                }
                
                const float theta = angle_min + i * angle_inc;
                const float x = range * std::cos(theta) + lidar_offset_x_;
                const float y = range * std::sin(theta) + lidar_offset_y_;
                // if(printed < 3){
                //     RCLCPP_INFO(this->get_logger(),"Obstacle point: x=%.2f, y=%.2f",x,y);
                //     ++printed;
                // }

                const float x_odom = robot_x_ + x*std::cos(robot_theta_) - y*std::sin(robot_theta_);
                const float y_odom = robot_y_ + x*std::sin(robot_theta_) + y*std::cos(robot_theta_);
                // obstacle.emplace_back(x_odom, y_odom);

                // plot the robot and obstacle in a map with 10m*10m and 0.1m/cell resolution
                const int col = static_cast<int>((x_odom - origin_x_) / resolution_);
                const int row = static_cast<int>((y_odom - origin_y_) / resolution_);

                const int robot_col = static_cast<int>((robot_x_ - origin_x_) / resolution_);
                const int robot_row = static_cast<int>((robot_y_ - origin_y_) / resolution_);

                if (row >= 0 && row < height_ &&
                    col >= 0 && col < width_ &&
                    robot_row >= 0 && robot_row < height_ &&
                    robot_col >= 0 && robot_col < width_){
                        
                    BresenhamLine(robot_col, robot_row, col, row, grid_);
                    // obstacle_cells.emplace_back(col, row);
                }
            }
            nav_msgs::msg::OccupancyGrid map_msg = createMapMessage();
            map_pub_->publish(map_msg);

            // RCLCPP_INFO(this->get_logger(), "detected: %zu points", obstacle.size());
            // %zu for size_t

            // if (!obstacle.empty()){
            //     RCLCPP_INFO(this->get_logger(), "First odom obstacle: (%d, %d)", obstacle_cells[0].first, obstacle_cells[0].second);
            // }
        }
        
        void BresenhamLine(int start_col, int start_row, int end_col, int end_row,std::vector<int8_t>& grid){
            int x = start_col;
            int y = start_row;

            const int dx = std::abs(end_col - start_col);
            const int dy = std::abs(end_row - start_row);

            const int sx = (start_col < end_col) ? 1 : -1;
            const int sy = (start_row < end_row) ? 1 : -1;

            int error = dx - dy;

            while (true){
                // Reached obstacle endpoint
                if (x == end_col && y == end_row){
                    const int index = y * width_ + x;
                    grid[index] = 100;
                    break;
                }

                // Mark current cell as free
                if (x >= 0 && x < width_ && y >= 0 && y < height_){
                    const int index = y * width_ + x;

                    // Don't overwrite an occupied cell
                    if (grid[index] != 100) grid[index] = 0;
                }

                const int e2 = 2 * error;

                if (e2 > -dy){
                    error -= dy;
                    x += sx;
                }

                if (e2 < dx){
                    error += dx;
                    y += sy;
                }
            }
        }

        nav_msgs::msg::OccupancyGrid createMapMessage(){
            nav_msgs::msg::OccupancyGrid map_msg;
            map_msg.header.stamp = this->now();
            map_msg.header.frame_id = "odom";

            map_msg.info.height = height_;
            map_msg.info.width = width_;
            map_msg.info.resolution = resolution_;

            map_msg.info.origin.position.x = origin_x_;
            map_msg.info.origin.position.y = origin_y_;
            map_msg.info.origin.position.z = 0.0;

            map_msg.info.origin.orientation.x = 0.0;
            map_msg.info.origin.orientation.y = 0.0;
            map_msg.info.origin.orientation.z = 0.0;
            map_msg.info.origin.orientation.w = 1.0;

            map_msg.data = grid_;

            return map_msg;
        }
        
        // parameters
        double robot_x_ = 0.0;
        double robot_y_ = 0.0;
        double robot_theta_ = 0.0;

        const double lidar_offset_x_ = 0.2;
        const double lidar_offset_y_ = 0.0;

        const float origin_x_ = -5.0f;
        const float origin_y_ = -5.0f;
        const float resolution_ = 0.1f;
        const int width_  = 100;
        const int height_ = 100;
        std::vector<int8_t> grid_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}