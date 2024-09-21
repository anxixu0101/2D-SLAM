// map_builder.hpp
#ifndef MAP_BUILDER_HPP
#define MAP_BUILDER_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>  // For pcl::transformPointCloud
#include "lidar_sub.hpp"
#include "odom_sub.hpp"
#include "front_end.hpp"
class MapBuilder : public rclcpp::Node {
public:
    MapBuilder();

    void buildMap(const nav_msgs::msg::Odometry::SharedPtr odom, 
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud,int lidar_frame);
private:
    void publishMap();  // 负责发布地图

private:

    std::shared_ptr<FrontEnd> front_end_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
};



#endif