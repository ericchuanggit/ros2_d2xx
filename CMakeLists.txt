cmake_minimum_required(VERSION 3.8)
project(ros2_d2xx)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include/D2XX)

add_executable(pub_sub src/pub_sub.cpp)

ament_target_dependencies(pub_sub rclcpp std_msgs)

install(TARGETS pub_sub DESTINATION lib/${PROJECT_NAME})

ament_package()
