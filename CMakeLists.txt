cmake_minimum_required(VERSION 3.14)
project(smart_nav_bot)

set(smart_nav_bot_MAJOR_VERSION 0)
set(smart_nav_bot_MINOR_VERSION 0)
set(smart_nav_bot_PATCH_VERSION 0)
set(smart_nav_bot_VERSION
  ${smart_nav_bot_MAJOR_VERSION}.${smart_nav_bot_MINOR_VERSION}.${smart_nav_bot_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LocationSave.srv"
  "srv/MoveToRoom.srv"
  "srv/GetRoomName.srv"
  "action/NavigateToRoom.action"
  DEPENDENCIES geometry_msgs
)


add_executable(basic_mover src/basic_mover.cpp)
ament_target_dependencies(basic_mover rclcpp geometry_msgs nav_msgs tf2)

add_executable(odom_sub src/odom_sub.cpp)
ament_target_dependencies(odom_sub rclcpp nav_msgs)

add_executable(location_manager src/location_manager.cpp)
ament_target_dependencies(location_manager rclcpp geometry_msgs nav2_msgs rclcpp_action)

add_executable(navigation_manager src/navigation_manager.cpp)
ament_target_dependencies(navigation_manager rclcpp geometry_msgs nav2_msgs rclcpp_action)


rosidl_target_interfaces(location_manager ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(navigation_manager ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS basic_mover
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS odom_sub
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS location_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS navigation_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS smart_nav_bot/voice_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS smart_nav_bot/room_saver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()