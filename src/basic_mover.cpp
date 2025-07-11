#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class BasicMoverPublisher : public rclcpp::Node
{

public:
    BasicMoverPublisher()
     : Node("mover_publisher")
     {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&BasicMoverPublisher::move_forward, this));
        initial_time = this->now();
     }


private:
     void move_forward()
     {
         geometry_msgs::msg::Twist cmd;
         if (sides == 4){
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            publisher_->publish(cmd);
            return;
         }

         current_time = this->now();
         switch (state)
         {
         case 0:
            if (elapsedTime(initial_time, current_time) < 2.0){
               cmd.linear.x = 0.2;
               cmd.angular.z = 0.0;
            }

            else{
               state = 1;
            }
            break;

         case 1:
            if (elapsedTime(initial_time, current_time) < 4.0){
               cmd.linear.x = 0.0;
               cmd.angular.z = 1.0;
            }

            else{
               state = 0;
               initial_time = this->now();
               sides++;
            }
            break;
         
         default:
            break;
         }

         publisher_->publish(cmd);
         RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %f, angular.z = %f", cmd.linear.x, cmd.angular.z);
     }

     double elapsedTime(rclcpp::Time iTime,rclcpp::Time cTime){
         rclcpp::Duration dTime = cTime - iTime;
         double seconds = dTime.seconds();
         return seconds;
     }

     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
     rclcpp::TimerBase::SharedPtr timer_;
     rclcpp::Time initial_time;
     rclcpp::Time current_time;
     int sides = 0;
     int state;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicMoverPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}