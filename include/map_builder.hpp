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
    MapBuilder(std::shared_ptr<OdomSubscriber> odom_sub, std::shared_ptr<LidarListener> lidar_sub);

    void buildMap();
private:
    void publishMap();  // 负责发布地图

private:

    std::shared_ptr<FrontEnd> front_end_;
    std::shared_ptr<OdomSubscriber> odom_sub_;  // 里程计订阅节点
    std::shared_ptr<LidarListener> lidar_sub_;  // 雷达订阅节点
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;  // 用于存储全局点云地图

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;  // 地图发布者

private:
    rclcpp::Time last_odom_stamp_;
    rclcpp::Time last_lidar_stamp_;
    Eigen::Matrix4d robot_pose_ =Eigen::Matrix4d::Identity();
    double robot_pose_x_ =0.0;
    double robot_pose_y_ =0.0;
    Eigen::Matrix3d angle_matrix_ = Eigen::Matrix3d::Identity(); //旋转矩阵
};



#endif