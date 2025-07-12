#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class BasicMoverPublisher : public rclcpp::Node
{

public:
    BasicMoverPublisher()
     : Node("mover_publisher")
     {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&BasicMoverPublisher::move_forward, this, std::placeholders::_1));
     }


private:
     void move_forward(const nav_msgs::msg::Odometry::SharedPtr msg)
     {
         geometry_msgs::msg::Twist cmd;

         if (start == 0){
            initial_position = msg->pose.pose.position;
            initial_rotation = msg->pose.pose.orientation;
            start = 1;
         }

         if (sides == 4){
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            publisher_->publish(cmd);
            return;
         }

         auto current_position = msg->pose.pose.position;
         auto current_rotation = msg->pose.pose.orientation;
         switch (state)
         {
         case 0:
            if (distance_calc(initial_position, current_position) < 0.5){
               cmd.linear.x = 0.2;
               cmd.angular.z = 0.0;
            }

            else{
               state = 1;
               start = 0;
            }
            break;

         case 1: {
            double initial_yaw = quaternion_to_yaw(initial_rotation);
            double current_yaw = quaternion_to_yaw(current_rotation);
            double angle_diff = current_yaw - initial_yaw;

            if (angle_diff > M_PI) {angle_diff -= 2 * M_PI;}
            if (angle_diff < -M_PI) {angle_diff += 2 * M_PI;}

            //RCLCPP_INFO(this->get_logger(), "  Turning. Initial Yaw: %.2f, Current Yaw: %.2f, Angle Diff: %.2f", initial_yaw, current_yaw, angle_diff);

            if (std::abs(std::abs(angle_diff) - M_PI/2) > 0.05){
               RCLCPP_INFO(this->get_logger(), "angle diff: %f", angle_diff);
               cmd.linear.x = 0.0;
               cmd.angular.z = 0.7;
            }

            else{
               state = 0;
               start = 0;
               sides++;
            }
            break;
         }
         
         default:
            break;
         }

         publisher_->publish(cmd);
         //RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %f, angular.z = %f", cmd.linear.x, cmd.angular.z);
     }

     double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q){
         tf2::Quaternion quat(q.x, q.y, q.z, q.w);
         tf2::Matrix3x3 m(quat);
         double roll, pitch, yaw;
         m.getEulerYPR(yaw, pitch, roll);
         RCLCPP_INFO(this->get_logger(), "Quaternion (x=%.3f, y=%.3f, z=%.3f, w=%.3f) -> Yaw: %.3f rad (%.1f deg)",
                     q.x, q.y, q.z, q.w, yaw, yaw * 180.0 / M_PI);
         return yaw;
     }

     double distance_calc(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2){
         return std::sqrt(std::pow(p1.y - p2.y, 2) + std::pow(p1.x - p2.x, 2));
     }

     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
     int sides = 0;
     // 0 is straight and 1 is turn
     int state = 0;
     int start = 0;
     geometry_msgs::msg::Point initial_position;
     geometry_msgs::msg::Quaternion initial_rotation;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicMoverPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}