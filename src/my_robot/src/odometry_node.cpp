#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

const double pi = std::acos(-1.0);

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode()
        : Node("odometry_node")
    {
        joint_state_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                std::bind(
                    &OdometryNode::jointStateCallback,
                    this,
                    std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom",10);
        RCLCPP_INFO(this->get_logger(), "Odometry node started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
{
        if (msg->name.size() < 2 ||
            msg->position.size() < 2 ||
            msg->velocity.size() < 2){
            return;
        }

        if (first_measurement_){
            previous_position_l_ = msg->position[0];
            previous_position_r_ = msg->position[1];
            first_measurement_ = false;

            return;
        }

        const double distance_l = wheel_radius_*(msg->position[0]-previous_position_l_);
        const double distance_r = wheel_radius_*(msg->position[1]-previous_position_r_);        

        const double delta_s = (distance_r + distance_l)/2;
        const double delta_theta = (distance_r - distance_l)/wheel_separation_;
        const double theta_mid = theta_ + delta_theta/2.0; 

        x_ += delta_s * std::cos(theta_mid);
        y_ += delta_s * std::sin(theta_mid);
        theta_ += delta_theta;

        previous_position_l_ = msg->position[0];
        previous_position_r_ = msg->position[1];
        theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

        const double velocity = wheel_radius_*(msg->velocity[1]+msg->velocity[0])/2;
        const double angular_velocity = wheel_radius_*(msg->velocity[1]-msg->velocity[0])/wheel_separation_;

        nav_msgs::msg::Odometry  odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(theta_/2.0);
        odom_msg.pose.pose.orientation.w = std::cos(theta_/2.0);

        odom_msg.twist.twist.linear.x = velocity;
        odom_msg.twist.twist.angular.z = angular_velocity;

        // Pose covariance
        odom_msg.pose.covariance[0]  = 0.0025;  // x:   (0.05 m)^2
        odom_msg.pose.covariance[7]  = 0.0025;  // y:   (0.05 m)^2
        odom_msg.pose.covariance[35] = 0.0004;  // yaw: (0.02 rad)^2

        // Twist covariance
        odom_msg.twist.covariance[0]  = 0.0001; // vx:  (0.01 m/s)^2
        odom_msg.twist.covariance[7]  = 0.0001; // vy:  (0.01 m/s)^2
        odom_msg.twist.covariance[35] = 0.0004; // vyaw:(0.02 rad/s)^2
        
        odom_pub_->publish(odom_msg);
        // RCLCPP_INFO(
        // this->get_logger(),
        // "Pose: x=%.3f m, y=%.3f m, theta=%.3f rad",
        // x_, y_, theta_);
    }

    // Robot Geometry
    const double wheel_radius_ = 0.1;
    const double wheel_separation_ = 0.46;

    // Odometry state
    double previous_position_l_;
    double previous_position_r_;
    bool first_measurement_ = true;
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}