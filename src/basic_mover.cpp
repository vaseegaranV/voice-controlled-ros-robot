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
         if (cycle_ < 10)
         {
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
         }

         else if (cycle_ < 20)
         {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5;
         }

         else
         {
            cycle_ = 0;
         }

         publisher_->publish(cmd);
         cycle_++;
     }

     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
     rclcpp::TimerBase::SharedPtr timer_;
     int cycle_ = 0;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicMoverPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}