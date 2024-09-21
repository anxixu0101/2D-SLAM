#ifndef ODOM_LIDAR_SYNC_HPP
#define ODOM_LIDAR_SYNC_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "map_builder.hpp"
#include <deque>

class OdomLidarSync : public rclcpp::Node {
public:
    OdomLidarSync();

private:
    // Odom 和 Lidar 数据缓存
    std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_buffer_;
    std::deque<sensor_msgs::msg::LaserScan::SharedPtr> lidar_buffer_;

    // Odom 数据回调函数
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Lidar 数据回调函数
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // 手动同步 Odom 和 Lidar 数据
    void synchronizeData();

    // 将 LaserScan 转换为 PCL 点云
    void convertLaserScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr& msg);

   
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    std::shared_ptr<MapBuilder> map_builder_;
    int lidar_frame_ =0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_lidar_cloud_;
};

#endif  // ODOM_LIDAR_SYNC_HPP
