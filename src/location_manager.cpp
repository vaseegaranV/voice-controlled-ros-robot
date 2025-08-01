#include <memory>
#include <chrono>
#include <map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "smart_nav_bot/srv/move_to_room.hpp"
#include "smart_nav_bot/srv/location_save.hpp"
#include "smart_nav_bot/srv/get_room_name.hpp"

class LocationManager : public rclcpp::Node{
    public:
        LocationManager() : Node("location_manager"){
            room_locations = std::map<std::string, geometry_msgs::msg::Pose>();
            location_save_service_ = this->create_service<smart_nav_bot::srv::LocationSave>("save_location", std::bind(&LocationManager::save_location_callback, this, std::placeholders::_1, std::placeholders::_2));
            navigate_service_ = this->create_service<smart_nav_bot::srv::MoveToRoom>("navigate_room", std::bind(&LocationManager::navigate_to_room_callback, this, std::placeholders::_1, std::placeholders::_2));
            nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
            room_name_service = this->create_service<smart_nav_bot::srv::GetRoomName>("get_room_names", std::bind(&LocationManager::get_rooms_callback, this, std::placeholders::_1, std::placeholders::_2));
        }
    
    private:
        std::map<std::string, geometry_msgs::msg::Pose> room_locations;
        rclcpp::Service<smart_nav_bot::srv::LocationSave>::SharedPtr location_save_service_;
        rclcpp::Service<smart_nav_bot::srv::MoveToRoom>::SharedPtr navigate_service_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;
        rclcpp::Service<smart_nav_bot::srv::GetRoomName>::SharedPtr room_name_service;
        
        void save_location_callback(const std::shared_ptr<smart_nav_bot::srv::LocationSave::Request> request, 
            std::shared_ptr<smart_nav_bot::srv::LocationSave::Response> response){
                room_locations[request->room_name] = request->pose;
                response->success = true;
                response->msg = "Room '" + request->room_name + "' saved successfully";

                RCLCPP_INFO(this->get_logger(), "Saved %s", request->room_name.c_str());
        }

        void navigate_to_room_callback(const std::shared_ptr<smart_nav_bot::srv::MoveToRoom::Request> request, 
            std::shared_ptr<smart_nav_bot::srv::MoveToRoom::Response> response){

                if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5))){
                    response->success = false;
                    response->msg = "Service not available";
                }
                
                if (room_locations.find(request->room_name) == room_locations.end()){
                    response->success = false;
                    response->msg = "Failed";
                    return;
                }

                auto pose = room_locations[request->room_name];

                auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
                goal_msg.pose.header.frame_id = "map";
                goal_msg.pose.header.stamp = this->now();
                goal_msg.pose.pose = pose;

                nav_action_client_->async_send_goal(goal_msg);

                response->success = true;
                response->msg = "Navigating to " + request->room_name;
    
                RCLCPP_INFO(this->get_logger(), "Navigating to room: %s", request->room_name.c_str());
        }

        void get_rooms_callback(const std::shared_ptr<smart_nav_bot::srv::GetRoomName::Request> /*request*/, 
            std::shared_ptr<smart_nav_bot::srv::GetRoomName::Response> response){
                
                std::vector<std::string> room_names;
                std::vector<geometry_msgs::msg::Pose> poses;
                
                for(const auto &pair:this->room_locations){
                    room_names.push_back(pair.first);
                    poses.push_back(pair.second);
                }

                response->room_names = room_names;
                response->poses = poses;

                RCLCPP_INFO(this->get_logger(), "Sending room details...");
        }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocationManager>());
    rclcpp::shutdown();
    return 0;
}