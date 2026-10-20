#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

class PathFollowerNode : public rclcpp::Node{
    public:
        PathFollowerNode() : Node("path_follower_node"){
            auto path_qos = rclcpp::QoS(1).transient_local();
            path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/planned_path",path_qos ,std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",10 ,std::bind(&PathFollowerNode::odomCallback, this, std::placeholders::_1));

            vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            control_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(50),
                    std::bind(&PathFollowerNode::controlLoop, this)); //run control loop in 20Hz
        }

    private:
        // parameters
        static constexpr double LOOKAHEAD_DISTANCE = 0.4;
        static constexpr double SLOW_DOWN_DISTANCE = 0.5;
        static constexpr double GOAL_TOLERANCE = 0.10;

        static constexpr double KP_ANGULAR = 1.5;

        static constexpr double MAX_ANGULAR_VEL = 1.0;
        static constexpr double MAX_LINEAR_VEL = 0.6;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

        // callbacks
        void pathCallback(const nav_msgs::msg::Path::SharedPtr msg){
            current_path_ = *msg;
            waypoint_index_ = 0;
            if (!current_path_.poses.empty()) received_path_ = true;
        }
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            robot_x_ = msg->pose.pose.position.x;
            robot_y_ = msg->pose.pose.position.y;

            double z = msg->pose.pose.orientation.z;
            double w = msg->pose.pose.orientation.w;

            robot_theta_ = 2.0 * std::atan2(z, w);
            received_odom_ = true;
        }
        void controlLoop(){
            if(!received_path_ || !received_odom_) return;

            if (current_path_.poses.empty()) return;

            // Check for goal first
            const double goal_distance = findDistance(current_path_, current_path_.poses.size()-1);

            if (goal_distance < GOAL_TOLERANCE){
                geometry_msgs::msg::Twist cmd;
                vel_pub_->publish(cmd);
                waypoint_index_ = current_path_.poses.size();

                return;
            }

            // Check the closest cell
            if (waypoint_index_ >= current_path_.poses.size()) {
                waypoint_index_ = current_path_.poses.size() - 1;
            }

            std::size_t closest_index = waypoint_index_;
            double closest_distance = std::numeric_limits<double>::max();

            for (std::size_t i = waypoint_index_; i < current_path_.poses.size(); ++i){
                const double distance = findDistance(current_path_, i);

                if (distance < closest_distance){
                    closest_distance = distance;
                    closest_index = i;
                }
            }
            waypoint_index_ = closest_index;

            // target the point which is LOOKAHEAD_DISTANCE away
            std::size_t target_index = waypoint_index_;

            double accumulated_distance = 0.0;

            for (std::size_t i = waypoint_index_; i + 1 < current_path_.poses.size(); ++i){
                const auto& current = current_path_.poses[i].pose.position;

                const auto& next = current_path_.poses[i + 1].pose.position;

                const double dx = next.x - current.x;
                const double dy = next.y - current.y;
                accumulated_distance += std::sqrt(dx * dx + dy * dy);
                target_index = i + 1;

                if (accumulated_distance >= LOOKAHEAD_DISTANCE) {
                    break;
                }
            }

            // move toward the target
            const auto& target = current_path_.poses[target_index].pose.position;
            double dx = target.x - robot_x_;
            double dy = target.y - robot_y_;

            double target_theta = std::atan2(dy, dx);
            double theta_error = target_theta - robot_theta_;
            theta_error = std::atan2(std::sin(theta_error), std::cos(theta_error));
            //normalize to [-π, π]
            
            double ang_vel = KP_ANGULAR * theta_error;
            ang_vel = std::clamp(ang_vel, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

            // Reduce forward velocity when the robot is not pointing toward the target / when the robot is close to the goal.
            const double heading_scale = std::max(0.0, std::cos(theta_error));
            double goal_scale = std::clamp(goal_distance / SLOW_DOWN_DISTANCE, 0.0, 1.0);

            double lin_vel = MAX_LINEAR_VEL * heading_scale * goal_scale;

            geometry_msgs::msg::Twist cmd;
            cmd.angular.z = ang_vel;
            cmd.linear.x = lin_vel;

            vel_pub_->publish(cmd);
        }

        double findDistance(nav_msgs::msg::Path path, int index){
            const auto& point = path.poses[index].pose.position;
            const double dx = point.x - robot_x_;
            const double dy = point.y - robot_y_;
            const double distance = std::sqrt(dx * dx + dy * dy);

            return distance;
        }

        nav_msgs::msg::Path current_path_;
        double robot_x_ = 0.0;
        double robot_y_ = 0.0;
        double robot_theta_ = 0.0;

        std::size_t waypoint_index_ = 0;
        bool received_path_ = false;
        bool received_odom_ = false;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollowerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}