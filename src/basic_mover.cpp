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
     }


private:
     void move_forward()
     {
         geometry_msgs::msg::Twist cmd;
         current_time_ = this->now().seconds();
         if (current_time_ - initial_time_ < 3)
         {
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
         }

         else if (current_time_ - initial_time_ < 4)
         {
            cmd.linear.x = 0.0;
            cmd.angular.z = 1.570796327;
         }

         else
         {
            initial_time_ = current_time_;
         }

         publisher_->publish(cmd);
         RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %f, angular.z = %f", cmd.linear.x, cmd.angular.z);
     }

     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
     rclcpp::TimerBase::SharedPtr timer_;
     int initial_time_ = this->now().seconds();
     int current_time_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicMoverPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}