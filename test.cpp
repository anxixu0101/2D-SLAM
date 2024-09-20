#include "include/lidar_sub.hpp"
#include "include/odom_sub.hpp"
#include "include/map_builder.hpp"
int main(int argc, char *argv[])
{
        rclcpp::init(argc, argv);

    // 创建 Odom 和 Lidar 订阅节点
    auto odom_sub = std::make_shared<OdomSubscriber>();
    auto lidar_sub = std::make_shared<LidarListener>();

    // 创建 MapBuilder，传入 Odom 和 Lidar 订阅节点
    auto map_builder = std::make_shared<MapBuilder>(odom_sub, lidar_sub);

    // 定时执行地图拼接
    rclcpp::Rate rate(1.0);  // 每秒拼接一次
    while (rclcpp::ok()) {
        map_builder->buildMap();
        rclcpp::spin_some(odom_sub);
        rclcpp::spin_some(lidar_sub);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
