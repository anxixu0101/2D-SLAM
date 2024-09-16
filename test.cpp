#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarListener : public rclcpp::Node
{
public:
    LidarListener() : Node("lidar_listener")
    {
        // 订阅雷达数据的话题
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_scan", 10, std::bind(&LidarListener::scanCallback, this, std::placeholders::_1));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received a LaserScan message with %zu ranges", msg->ranges.size());
        // 在此处理接收到的雷达数据
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    // 初始化ROS 2节点
    rclcpp::init(argc, argv);

    // 创建并运行节点
    rclcpp::spin(std::make_shared<LidarListener>());

    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}
