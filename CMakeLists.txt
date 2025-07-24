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

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LocationSave.srv"
  "srv/MoveToRoom.srv"
  DEPENDENCIES geometry_msgs
)


add_executable(basic_mover src/basic_mover.cpp)
ament_target_dependencies(basic_mover rclcpp geometry_msgs nav_msgs tf2)

add_executable(odom_sub src/odom_sub.cpp)
ament_target_dependencies(odom_sub rclcpp nav_msgs)

install(TARGETS basic_mover
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS odom_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()