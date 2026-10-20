#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "my_robot/astar_planner.hpp"

#include <cmath>
#include <vector>
#include <utility>

class LidarNode : public rclcpp::Node
{
    public:
        LidarNode() : Node("lidar_node"), planner_(width_, height_)
        {
            grid_.assign(width_ * height_, -1);

            goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
            ("goal_pose", 10, std::bind(&LidarNode::goalCallback, this, std::placeholders::_1));

            lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
            ("/scan", 10, std::bind(&LidarNode::scanCallback, this, std::placeholders::_1));
            // create_subscription<MessageType>(topic_name, queue_size, callback)

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&LidarNode::odomCallback, this, std::placeholders::_1));

            map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

            //keep the latest message.
            auto path_qos = rclcpp::QoS(1).transient_local();
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", path_qos);
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
                goal_x_ = msg->pose.position.x;
                goal_y_ = msg->pose.position.y;

                received_goal_ = true;
                replan_requested_ = true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "New goal received: x=%.2f, y=%.2f",
                    goal_x_,
                    goal_y_
                );
        }
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
            // std::vector<std::pair<float, float>> obstacle;
            
            for(std::size_t i = 0; i < msg->ranges.size(); ++i){
                const float range = msg->ranges[i];
                const float theta = angle_min + i * angle_inc;

                bool hit_obstacle = true;
                float used_range = range;

                if (!std::isfinite(range)){
                    hit_obstacle = false;
                    used_range = msg->range_max;
                }else if (range < msg->range_min || range > msg->range_max){
                    continue;
                }
                
                const float lidar_x_odom = robot_x_
                    + lidar_offset_x_ * std::cos(robot_theta_)
                    - lidar_offset_y_ * std::sin(robot_theta_);

                const float lidar_y_odom = robot_y_
                    + lidar_offset_x_ * std::sin(robot_theta_)
                    + lidar_offset_y_ * std::cos(robot_theta_);

                const float world_angle = robot_theta_ + theta;

                const float x_odom = lidar_x_odom + used_range * std::cos(world_angle);

                const float y_odom = lidar_y_odom + used_range * std::sin(world_angle);

                // obstacle.emplace_back(x_odom, y_odom);

                // bool printed = false;
                // if (!printed){
                //     RCLCPP_INFO(this->get_logger(), "robot_theta=%.2f, lidar_theta=%.2f, range=%.2f, x_odom=%.2f, y_odom=%.2f", robot_theta_, theta, range, x_odom,y_odom);
                //     printed = true;
                // }

                // plot the robot and obstacle in a map with 10m*10m and 0.1m/cell resolution
                std::pair<int,int> obstacle_cell = worldToGrid(x_odom, y_odom);
                std::pair<int,int> lidar_cell = worldToGrid(lidar_x_odom, lidar_y_odom);

                if (lidar_cell.first >= 0 && lidar_cell.first < height_ &&
                    lidar_cell.second >= 0 && lidar_cell.second < width_){
                    
                    // Add the cells on lidar path to grid
                    BresenhamLine(lidar_cell, obstacle_cell, grid_, hit_obstacle);
                }
            }
            nav_msgs::msg::OccupancyGrid map_msg = createMapMessage();
            map_pub_->publish(map_msg);

            // RCLCPP_INFO(this->get_logger(), "detected: %zu points", obstacle.size());
            // %zu for size_t

            // if (!obstacle.empty()){
            //     RCLCPP_INFO(this->get_logger(), "First odom obstacle: (%.1f, %.1f)", obstacle[0].first, obstacle[0].second);
            // }

            // Planner

            if (received_goal_ && replan_requested_){
                std::pair<int,int> robot_cell = worldToGrid(robot_x_, robot_y_);

                std::pair<int,int> goal_cell = worldToGrid(goal_x_, goal_y_);

                auto path = planner_.findPath(grid_, robot_cell, goal_cell);

                if(!path.empty()){
                    auto path_msg = createPathMessage(path);
                    path_pub_->publish(path_msg);
                    replan_requested_ = false;

                    RCLCPP_INFO(this->get_logger(), "Path found!");
                }else{
                    RCLCPP_WARN(
                        this->get_logger(),
                        "No path found from (%d,%d) to (%d,%d)",
                        robot_cell.first,
                        robot_cell.second,
                        goal_cell.first,
                        goal_cell.second
                    );
                }
            }

        }
        
        void BresenhamLine(const std::pair<int,int>& start_cell, const std::pair<int,int>& end_cell, std::vector<int8_t>& grid, bool hit_obstacle){

            int x = start_cell.second;
            int y = start_cell.first;

            const int dx = std::abs(end_cell.second - start_cell.second);
            const int dy = std::abs(end_cell.first - start_cell.first);

            const int sx = (start_cell.second < end_cell.second) ? 1 : -1;
            const int sy = (start_cell.first < end_cell.first) ? 1 : -1;

            int error = dx - dy;

            while (true){
                // ray left the map
                if (x < 0 || x >= width_ || y < 0 || y >= height_){
                    break;
                }

                const int index = y * width_ + x;

                // Reached obstacle endpoint
                if (x == end_cell.second && y == end_cell.first){

                    if (hit_obstacle){
                        grid[index] = 100;
                    }else{
                        grid[index] = 0;
                    }
                    break;
                }

                grid[index] = 0;

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

        std::pair<int,int> worldToGrid(double x, double y) const {
            int col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
            int row = static_cast<int>(std::floor((y - origin_y_) / resolution_));

            return std::pair<int,int>{row, col};
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
        
        nav_msgs::msg::Path createPathMessage(const std::vector<std::pair<int,int>>& path){
            nav_msgs::msg::Path msg;

            msg.header.stamp = this->now();
            msg.header.frame_id = "odom";
            msg.poses.reserve(path.size()); //reserve space for poses

            for(const auto& p:path){
                geometry_msgs::msg::PoseStamped temp;
                // calculate the center of the grid
                const float x = origin_x_ + (p.second + 0.5) * resolution_;
                const float y = origin_y_ + (p.first + 0.5) * resolution_;

                temp.header = msg.header;
                temp.pose.position.x = x;
                temp.pose.position.y = y;
                temp.pose.position.z = 0.0;
                temp.pose.orientation.w = 1;

                msg.poses.push_back(temp);
            }

            return msg;
        }

        // parameters
        double robot_x_ = 0.0;
        double robot_y_ = 0.0;
        double robot_theta_ = 0.0;

        double goal_x_ = 0.0;
        double goal_y_ = 0.0;

        const double lidar_offset_x_ = 0.2;
        const double lidar_offset_y_ = 0.0;

        const float origin_x_ = -5.0f;
        const float origin_y_ = -5.0f;
        const float resolution_ = 0.1f;
        const int width_  = 100;
        const int height_ = 100;
        std::vector<int8_t> grid_;
        AStarPlanner planner_;

        bool received_goal_ = false;
        bool replan_requested_ = false;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}