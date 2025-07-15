#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class BasicMoverPublisher : public rclcpp::Node {

public:
    BasicMoverPublisher()
     : Node("mover_publisher")
     {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&BasicMoverPublisher::move_forward, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&BasicMoverPublisher::publish_msg, this));
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
            message_to_publish = cmd;
            return;
         }

         auto current_position = msg->pose.pose.position;
         auto current_rotation = msg->pose.pose.orientation;
         switch (state)
         {
         case 0:
            if (distance_calc(initial_position, current_position) < target_distance){
               cmd.angular.z = 0.0;
               error = target_distance - distance_calc(initial_position, current_position);
               RCLCPP_INFO(get_logger(), "ERROR: %f", error);
               double pid = caluculate_pid(error, prev_error_0, integral_0, state, clamp_val_0);
               cmd.linear.x = pid;
            }

            else{
               state = 1;
               start = 0;
               prev_error_0 = 0;
               integral_0 = 0;
               prev_error_1 = 0;
               integral_1 = 0;
            }
            break;

         case 1: {
            double initial_yaw = quaternion_to_yaw(initial_rotation);
            double current_yaw = quaternion_to_yaw(current_rotation);

            double target_yaw = initial_yaw + M_PI / 2.0;
            
            while (target_yaw > M_PI) {target_yaw -= M_PI * 2;}
            while (target_yaw <= -M_PI) {target_yaw += M_PI * 2;}

            double angle_error = target_yaw - current_yaw;

            if (angle_error > M_PI) {angle_error -= 2 * M_PI;}
            if (angle_error <= -M_PI) {angle_error += 2 * M_PI;}
            static int settle_counter;

            if (std::abs(angle_error) > 0.03){
               RCLCPP_INFO(this->get_logger(), "angle error: %f", angle_error);
               settle_counter = 0;
               cmd.linear.x = 0.0;
               double pid = caluculate_pid(angle_error, prev_error_1, integral_1, state, clamp_val_1);
               cmd.angular.z = pid;
            }

            else{
               settle_counter++;
               if (settle_counter > 5) {
                  state = 0;
                  start = 0;
                  prev_error_0 = 0;
                  integral_0 = 0;
                  prev_error_1 = 0;
                  integral_1 = 0;
                  sides++;
                  settle_counter = 0;
               }
            }
            break;
         }
         
         default:
            break;
         }

         message_to_publish = cmd;
         //RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %f, angular.z = %f", cmd.linear.x, cmd.angular.z);
     }

     double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q){
         tf2::Quaternion quat(q.x, q.y, q.z, q.w);
         tf2::Matrix3x3 m(quat);
         double roll, pitch, yaw;
         m.getEulerYPR(yaw, pitch, roll);
         return yaw;
     }

     double distance_calc(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2){
         return std::sqrt(std::pow(p1.y - p2.y, 2) + std::pow(p1.x - p2.x, 2));
     }

     void publish_msg(){
         publisher_->publish(message_to_publish);
     }

     double caluculate_pid(double error, double &prev_error, double &integral, int state, const double clamp_val){
         double Kp = 0.0;
         double Ki = 0.0;
         double Kd = 0.0;

         switch (state)
         {
         //For moving forward
         case 0:
            Kp = 2.7;
            Ki = 0.0;
            Kd = 0.1;
            break;
         
         //For turning
         case 1:
            Kp = 0.6;
            Ki = 0.1;
            Kd = 0.1;
            break;
         
         default:
            break;
         }
         

         double derivative = error - prev_error;
         double pid_val = Kp * error + Ki * integral + Kd * derivative;

         if (std::abs(pid_val) < clamp_val || error * pid_val < 0)
         {
            integral += error;
         }
         

         prev_error = error;

         return std::clamp(Kp * error + Ki * integral + Kd * derivative, -clamp_val, clamp_val);
     }

     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
     rclcpp::TimerBase::SharedPtr timer_;
     int sides = 0;
     // 0 is straight and 1 is turn
     int state = 0;
     int start = 0;
     geometry_msgs::msg::Point initial_position;
     geometry_msgs::msg::Quaternion initial_rotation;
     geometry_msgs::msg::Twist message_to_publish;

     double error;
     double prev_error_0 = 0;
     double integral_0 = 0;
     double prev_error_1 = 0;
     double integral_1 = 0;
     const double clamp_val_0 = 0.8;
     const double clamp_val_1 = 0.8;
     const double target_distance = 0.5;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicMoverPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}