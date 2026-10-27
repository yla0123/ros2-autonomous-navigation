#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

class PathFollowerNode : public rclcpp::Node{
    public:
        PathFollowerNode() : Node("path_follower_node"){
            auto path_qos = rclcpp::QoS(1).transient_local();
            path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/planned_path",path_qos ,std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",10 ,std::bind(&PathFollowerNode::odomCallback, this, std::placeholders::_1));

            goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PathFollowerNode::goalCallback, this,std::placeholders::_1));

            vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);

            control_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(50),
                    std::bind(&PathFollowerNode::controlLoop, this)); //run control loop in 20Hz

            // Delcare parameters
            kp_angular_ = this->declare_parameter<double>("kp_angular", 1.5);
            max_angular_vel_ = this->declare_parameter<double>("max_angular_vel", 1.0);
            max_linear_vel_ = this->declare_parameter<double>("max_linear_vel", 0.6);
            lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 0.4);
            goal_tolerance_ = this->declare_parameter<double>("goal_tolerance",0.10);
            slow_down_distance_ = this->declare_parameter<double>("slow_down_distance",0.6);
        }

    private:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;

        rclcpp::TimerBase::SharedPtr control_timer_;

        // callbacks
        void pathCallback(const nav_msgs::msg::Path::SharedPtr msg){
            current_path_ = *msg;
            waypoint_index_ = 0;
            if (!current_path_.poses.empty()){
                received_path_ = true;
                //RCLCPP_INFO(this->get_logger(), "Path received");
            }           
        }
        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr /*msg*/){
            goal_reached_published_ = false;
            // Old path is no longer valid for the new goal
            received_path_ = false;
            current_path_.poses.clear();
            waypoint_index_ = 0;
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

            if (goal_distance < goal_tolerance_){
                geometry_msgs::msg::Twist cmd;
                vel_pub_->publish(cmd);
                waypoint_index_ = current_path_.poses.size();

                if(!goal_reached_published_){
                    std_msgs::msg::Bool msg;
                    msg.data = true;
                    goal_reached_pub_->publish(msg);

                    goal_reached_published_ = true;

                    RCLCPP_INFO(this->get_logger(), "Goal reached");
                    }
                return;
            }

            // Find the closest cell on the path
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
                accumulated_distance += std::hypot(dx, dy);
                target_index = i + 1;

                if (accumulated_distance >= lookahead_distance_) {
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
            
            double ang_vel = kp_angular_ * theta_error;
            ang_vel = std::clamp(ang_vel, -max_angular_vel_, max_angular_vel_);

            // Reduce forward velocity when the robot is not pointing toward the target / when the robot is close to the goal.
            const double heading_scale = std::max(0.0, std::cos(theta_error));
            double goal_scale = std::clamp(goal_distance / slow_down_distance_, 0.0, 1.0);

            double lin_vel = max_linear_vel_ * heading_scale * goal_scale;

            geometry_msgs::msg::Twist cmd;
            cmd.angular.z = ang_vel;
            cmd.linear.x = lin_vel;
            vel_pub_->publish(cmd);
        }

        double findDistance(nav_msgs::msg::Path path, int index){
            const auto& point = path.poses[index].pose.position;
            const double dx = point.x - robot_x_;
            const double dy = point.y - robot_y_;
            const double distance = std::hypot(dx, dy);
            return distance;
        }

        nav_msgs::msg::Path current_path_;
        double robot_x_ = 0.0;
        double robot_y_ = 0.0;
        double robot_theta_ = 0.0;

        std::size_t waypoint_index_ = 0;
        bool received_path_ = false;
        bool received_odom_ = false;
        bool goal_reached_published_ = false;

        double kp_angular_;
        double max_angular_vel_;
        double max_linear_vel_;
        double lookahead_distance_;
        double goal_tolerance_;
        double slow_down_distance_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollowerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}