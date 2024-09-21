#include "include/lidar_sub.hpp"
#include "include/odom_sub.hpp"
#include "include/map_builder.hpp"
#include "include/time_sync.hpp"
int main(int argc, char *argv[])
{
   rclcpp::init(argc, argv);
    
    auto node = std::make_shared<OdomLidarSync>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

