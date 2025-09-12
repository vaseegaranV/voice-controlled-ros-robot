#include <memory>
#include <chrono>
#include <map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "smart_nav_bot/srv/move_to_room.hpp"
#include "smart_nav_bot/srv/location_save.hpp"
#include "smart_nav_bot/srv/get_room_name.hpp"
#include "smart_nav_bot/action/navigate_to_room.hpp"

using NavigateToRoom = smart_nav_bot::action::NavigateToRoom;
using GoalHandleNavigateToRoom = rclcpp_action::ClientGoalHandle<NavigateToRoom>;

class LocationManager : public rclcpp::Node{
    public:
        LocationManager() : Node("location_manager"){
            room_locations = std::map<std::string, geometry_msgs::msg::Pose>();
            location_save_service_ = this->create_service<smart_nav_bot::srv::LocationSave>("save_location", std::bind(&LocationManager::save_location_callback, this, std::placeholders::_1, std::placeholders::_2));
            navigate_service_ = this->create_service<smart_nav_bot::srv::MoveToRoom>("navigate_room", std::bind(&LocationManager::navigate_to_room_callback, this, std::placeholders::_1, std::placeholders::_2));
            nav_action_client_ = rclcpp_action::create_client<smart_nav_bot::action::NavigateToRoom>(this, "navigate_to_room");
            room_name_service = this->create_service<smart_nav_bot::srv::GetRoomName>("get_room_names", std::bind(&LocationManager::get_rooms_callback, this, std::placeholders::_1, std::placeholders::_2));
            nav_goal_result_pub_ = this->create_publisher<std_msgs::msg::String>("nav_goal_result", 10);
        }
    
    private:
        std::map<std::string, geometry_msgs::msg::Pose> room_locations;
        rclcpp::Service<smart_nav_bot::srv::LocationSave>::SharedPtr location_save_service_;
        rclcpp::Service<smart_nav_bot::srv::MoveToRoom>::SharedPtr navigate_service_;
        rclcpp_action::Client<smart_nav_bot::action::NavigateToRoom>::SharedPtr nav_action_client_;
        rclcpp::Service<smart_nav_bot::srv::GetRoomName>::SharedPtr room_name_service;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_goal_result_pub_;
        
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
                    return;
                }
                
                if (room_locations.find(request->room_name) == room_locations.end()){
                    response->success = false;
                    response->msg = "Failed";
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "We are in navigateTO room callback");

                auto goal_msg = smart_nav_bot::action::NavigateToRoom::Goal();
                goal_msg.room_name = request->room_name;

                auto goal_options = rclcpp_action::Client<smart_nav_bot::action::NavigateToRoom>::SendGoalOptions();
                goal_options.goal_response_callback = std::bind(&LocationManager::goal_response_callback, this, std::placeholders::_1);
                goal_options.feedback_callback = std::bind(&LocationManager::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
                goal_options.result_callback = std::bind(&LocationManager::result_callback, this, std::placeholders::_1);

                nav_action_client_->async_send_goal(goal_msg, goal_options);

                response->success = true;
                response->msg = "Navigating to " + request->room_name;
    
                RCLCPP_INFO(this->get_logger(), "Navigating to room: %s", request->room_name.c_str());
        }

        void goal_response_callback(std::shared_ptr<GoalHandleNavigateToRoom> goal_handle){
            if (!goal_handle){
                RCLCPP_INFO(this->get_logger(), "Goal rejected");
            }
            
            else{
                RCLCPP_INFO(this->get_logger(), "Goal accepted");
            }
        }

        void feedback_callback(std::shared_ptr<GoalHandleNavigateToRoom> goal_handle, const std::shared_ptr<const NavigateToRoom::Feedback> feedback){
            RCLCPP_INFO(this->get_logger(), "%s", feedback->message.c_str());
        }

        void result_callback(const GoalHandleNavigateToRoom::WrappedResult &result){
            auto msg = std_msgs::msg::String();
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "SEnding to voice: Message: %s", result.result->message.c_str());
                    msg.data = result.result->message.c_str();
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "SEnding to voice: Goal was aborted");
                    msg.data = "Goal was aborted";
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "SEnding to voice: Goal was canceled");
                    msg.data = "Goal was canceled";
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "SEnding to voice: Unknown result code");
                    msg.data = "Unknown result code";
                    break;
            }

            nav_goal_result_pub_->publish(msg);
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