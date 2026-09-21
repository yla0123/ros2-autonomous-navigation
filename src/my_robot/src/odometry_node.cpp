#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

        RCLCPP_INFO(this->get_logger(), "Odometry node started.");
    }

private:
    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.size() < 2 ||
            msg->position.size() < 2 ||
            msg->velocity.size() < 2)
        {
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

        x_ += delta_s * std::cos(theta_);
        y_ += delta_s * std::sin(theta_);
        theta_ += delta_theta;

        previous_position_l_ = msg->position[0];
        previous_position_r_ = msg->position[1];

        RCLCPP_INFO(
        this->get_logger(),
        "Pose: x=%.3f m, y=%.3f m, theta=%.3f rad",
        x_, y_, theta_);
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
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}