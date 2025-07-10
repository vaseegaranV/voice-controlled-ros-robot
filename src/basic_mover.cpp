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
        initial_time = this->now().seconds();
     }


private:
     void move_forward()
     {
         geometry_msgs::msg::Twist cmd;
         if (sides_ == 4){
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            publisher_->publish(cmd);
            return;
         }

         current_time = this->now().seconds();
         if (state == 0 && current_time - initial_time < 3)
         {
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
         }

         else if (state == 0 && current_time - initial_time >= 3)
         {
            state = 1;
         }

         else if (state == 1 && current_time - initial_time < 6)
         {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5236;
         }

         else
         {
            state = 0;
            initial_time = this->now().seconds();
            sides_++;
         }

         publisher_->publish(cmd);
         RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %f, angular.z = %f", cmd.linear.x, cmd.angular.z);
     }

     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
     rclcpp::TimerBase::SharedPtr timer_;
     int initial_time;
     int current_time;
     int sides_ = 0;
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