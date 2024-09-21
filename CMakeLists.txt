cmake_minimum_required(VERSION 2.6)
project(TEST)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(TEST test.cpp map_builder.cpp front_end.cpp time_sync.cpp)

target_include_directories(TEST PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries (TEST ${PCL_LIBRARIES})
ament_target_dependencies(TEST rclcpp std_msgs sensor_msgs nav_msgs pcl_conversions PCL message_filters)

install(TARGETS TEST RUNTIME DESTINATION bin)


