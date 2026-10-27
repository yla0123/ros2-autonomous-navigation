#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"

#include <cmath>
#include <algorithm>

class EvaluationNode : public rclcpp::Node
{
    public:
        EvaluationNode()
        : Node("evaluation_node")
        {
            goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.10);

            goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(&EvaluationNode::goalCallback, this, std::placeholders::_1));

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&EvaluationNode::odomCallback, this, std::placeholders::_1));

            gt_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("gazebo_odom", 10, std::bind(&EvaluationNode::groundTruthCallback, this, std::placeholders::_1));

            goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>
            ("/goal_reached", 10, std::bind(&EvaluationNode::goalReachedCallback, this,std::placeholders::_1));

        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;

        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            goal_x_ = msg->pose.position.x;
            goal_y_ = msg->pose.position.y;

            received_goal_ = true;

            // New navigation run
            evaluation_complete_ = false;

            actual_distance_ = 0.0;
            goal_error_ = 0.0;
            have_previous_position_ = false;

            localization_squared_error_sum_ = 0.0;
            localization_sample_count_ = 0;
            max_localization_error_ = 0.0;

            RCLCPP_INFO(
            this->get_logger(),
            "New evaluation run started. Goal: (%.2f, %.2f)",
            goal_x_, goal_y_);
        }

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            robot_x_ = msg->pose.pose.position.x;
            robot_y_ = msg->pose.pose.position.y;

            if(!received_goal_ || evaluation_complete_) return;

            if(!have_previous_position_){
                previous_x_ = robot_x_;
                previous_y_ = robot_y_;
                have_previous_position_ = true;
                return;
            }

            actual_distance_ += std::sqrt(
                (robot_x_ - previous_x_) * (robot_x_ - previous_x_) +
                (robot_y_ - previous_y_) * (robot_y_ - previous_y_));

            previous_x_ = robot_x_;
            previous_y_ = robot_y_;

            goal_error_ = std::sqrt(
                (goal_x_ - robot_x_) * (goal_x_ - robot_x_) +
                (goal_y_ - robot_y_) * (goal_y_ - robot_y_));
            
            // localization error
            if (received_gt_odom_){
                double dx = robot_x_ - gt_x_;
                double dy = robot_y_ - gt_y_;

                double localization_error = std::sqrt(dx * dx + dy * dy);

                localization_squared_error_sum_ += 
                localization_error * localization_error;

                localization_sample_count_++;

                max_localization_error_ = std::max
                (max_localization_error_, localization_error);
            }

        }
        void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            gt_x_ = msg->pose.pose.position.x;
            gt_y_ = msg->pose.pose.position.y;

            received_gt_odom_ = true;
        }

        void goalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg){
            if(!msg->data || evaluation_complete_) return;

            evaluation_complete_ = true;

            double localization_rmse = 0.0;

            if(localization_sample_count_ > 0){
                localization_rmse = std::sqrt(
                    localization_squared_error_sum_ /
                    static_cast<double>(localization_sample_count_));
            }

            RCLCPP_INFO(this->get_logger(), "\n"
                "=== Navigation Evaluation ===\n"
                "Goal: (%.3f, %.3f)\n"
                "Final robot position: (%.3f, %.3f)\n"
                "Final goal error: %.3f m\n"
                "Actual traveled distance: %.3f m\n"
                "Localization RMSE: %.3f m\n"
                "Maximum localization error: %.3f m\n"
                "Localization samples: %zu\n"
                "=============================",
                goal_x_, goal_y_,
                robot_x_, robot_y_,
                goal_error_,
                actual_distance_,
                localization_rmse,
                max_localization_error_,
                localization_sample_count_);
        }
        double goal_x_ = 0.0;
        double goal_y_ = 0.0;
        double goal_tolerance_ = 0.10;  

        double robot_x_ = 0.0;
        double robot_y_ = 0.0;

        double gt_x_ = 0.0;
        double gt_y_ = 0.0;

        double previous_x_ = 0.0;
        double previous_y_ = 0.0;

        double actual_distance_ = 0.0;
        double goal_error_ = 0.0;
        double localization_error_ = 0.0;

        double localization_squared_error_sum_ = 0.0;
        double max_localization_error_ = 0.0;
        std::size_t localization_sample_count_ = 0;

        bool received_goal_ = false;
        bool received_gt_odom_ = false;
        bool have_previous_position_ = false;
        bool evaluation_complete_ = false;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EvaluationNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}