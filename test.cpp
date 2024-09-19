#include "include/lidar_sub.hpp"
#include "include/odom_sub.hpp"

int main(int argc, char *argv[])
{
    // 初始化ROS 2节点
    rclcpp::init(argc, argv);

    // 创建并运行节点
    //rclcpp::spin(std::make_shared<LidarListener>());
    auto node = std::make_shared<LidarListener>();
   // rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
