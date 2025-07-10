#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class OdomReader : public rclcpp::Node{
    public:
        OdomReader() : Node("odomReader"){
            subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&OdomReader::subscriptionCallOdom, this, std::placeholders::_1));
        }

    private:
        void subscriptionCallOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;

            double vx = msg->twist.twist.linear.x;
            double wz = msg->twist.twist.angular.z;

            RCLCPP_INFO(this->get_logger(), "X position %f | Y position %f | linear speed %f | angular speed %f", x, y, vx, wz);
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomReader>());
    rclcpp::shutdown();
    return 0;
}