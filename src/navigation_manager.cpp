#include <memory>
#include <chrono>
#include <map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "smart_nav_bot/action/navigate_to_room.hpp"
#include "smart_nav_bot/srv/get_room_name.hpp"

using NavigateToRoom = smart_nav_bot::action::NavigateToRoom;
using GetRoomName = smart_nav_bot::srv::GetRoomName;
using GoalHandleNavigateToRoom = rclcpp_action::ServerGoalHandle<NavigateToRoom>;
using namespace std::placeholders;


class NavigationManager : public rclcpp::Node{
    public:
        NavigationManager() : Node("navigation_manager"){
            get_name_client_ = this->create_client<GetRoomName>("get_room_names");
            nav2_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
            nav_action_server_ = rclcpp_action::create_server<NavigateToRoom>(this, "navigate_to_room", 
                std::bind(&NavigationManager::handle_goal, this, _1, _2),
                std::bind(&NavigationManager::handle_cancel, this, _1),
                std::bind(&NavigationManager::handle_accepted, this, _1));
        }

    private:
        rclcpp::Client<GetRoomName>::SharedPtr get_name_client_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_action_client_;
        rclcpp_action::Server<NavigateToRoom>::SharedPtr nav_action_server_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToRoom::Goal> goal){
            // Replace 'room_name' with the actual member of NavigateToRoom::Goal if different
            RCLCPP_INFO(this->get_logger(), "Received goal request for room: %s", goal->room_name.c_str());
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigateToRoom> goal_handle){
             RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
             return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleNavigateToRoom> goal_handle){
            std::thread(&NavigationManager::execute, this, goal_handle).detach();
        }

        void execute(const std::shared_ptr<GoalHandleNavigateToRoom> goal_handle){
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<NavigateToRoom::Feedback>();
            auto result = std::make_shared<NavigateToRoom::Result>();

            auto request = std::make_shared<GetRoomName::Request>();
            auto future = get_name_client_->async_send_request(request);

            if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
            {
                RCLCPP_INFO(this->get_logger(), "Timeout waiting for server");
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            auto response = future.get();
            auto names = response->room_names;
            auto poses = response->poses;

            if(names.size() != poses.size()){
                RCLCPP_INFO(this->get_logger(), "Name list and Room list don't match");
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            auto iterator = std::find(names.begin(), names.end(), goal->room_name);

            if (iterator == names.end()) {
                RCLCPP_INFO(this->get_logger(), "Room not in list");
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            int index = std::distance(names.begin(), iterator);
            auto pose = poses[index];

            if (!nav2_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_ERROR(this->get_logger(), "Nav2 NavigateToPose action server not available.");
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            nav2_msgs::action::NavigateToPose::Goal nav_goal;
            nav_goal.pose.header.frame_id = "map";
            nav_goal.pose.header.stamp = this->now();
            nav_goal.pose.pose = pose;

            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
            options.feedback_callback = [this, goal_handle](auto, auto feedback) {
                this->feedback_callback(goal_handle, feedback);
            };

            auto nav2_goal_future = nav2_action_client_->async_send_goal(
                nav_goal,
                options
            );
            auto nav2_goal_handle = nav2_goal_future.get();

            if (!nav2_goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send goal to Nav2.");
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            while (rclcpp::ok())
            {
                if (goal_handle->is_canceling())
                {
                    RCLCPP_ERROR(this->get_logger(), "Cancelling");
                    auto cancel_future = nav2_action_client_->async_cancel_goal(nav2_goal_handle);
                    cancel_future.wait();
                    result->success = false;
                    goal_handle->canceled(result);
                    return;
                }

                auto status = nav2_goal_handle->get_status();

                if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED ||
                    status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                    status == rclcpp_action::GoalStatus::STATUS_CANCELED)
                {
                    break;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            

            auto nav2_goal_result = nav2_action_client_->async_get_result(nav2_goal_handle);
            auto nav2_result = nav2_goal_result.get();

            if (nav2_result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Successfully navigated to room: %s", goal->room_name.c_str());
                result->success = true;
                goal_handle->succeed(result);
            }
            
            else{
                RCLCPP_WARN(this->get_logger(), "Navigation to room failed.");
                result->success = false;
                goal_handle->abort(result);
            }
            
        }

        void feedback_callback(std::shared_ptr<GoalHandleNavigateToRoom> goal_handle, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback){
            auto fb = std::make_shared<NavigateToRoom::Feedback>();
            fb->message = std::to_string(feedback->distance_remaining);
            goal_handle->publish_feedback(fb);
        }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationManager>());
    rclcpp::shutdown();
    return 0;
}