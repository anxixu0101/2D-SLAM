cmake_minimum_required(VERSION 2.6)
project(TEST)

find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(TEST test.cpp)

target_link_libraries (TEST ${PCL_LIBRARIES})
ament_target_dependencies(TEST rclcpp std_msgs sensor_msgs)

install(TARGETS TEST RUNTIME DESTINATION bin)


