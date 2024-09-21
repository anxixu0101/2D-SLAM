#ifndef LIDAR_SUB_HPP_
#define LIDAR_SUB_HPP_
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class LidarListener : public rclcpp::Node
{
public:
    LidarListener() : Node("lidar_listener")
    {
        // 订阅雷达数据的话题
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_scan", 10, std::bind(&LidarListener::scanCallback, this, std::placeholders::_1));
        last_lidar_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_points", 10);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetLidarData() 
    {
        return last_lidar_cloud_;
    }
    int GetLidarFrame(){ return lidar_frame_;}
  
private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        lidar_frame_++;
        last_lidar_cloud_->clear();
        
        // 遍历 LaserScan 数据，将其转换为 PCL 点云
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (std::isfinite(msg->ranges[i]))
            {
                float angle = msg->angle_min + i * msg->angle_increment;

                // 将极坐标 (r, θ) 转换为笛卡尔坐标 (x, y, z)
                pcl::PointXYZ point;
                point.x = msg->ranges[i] * std::cos(angle);
                point.y = msg->ranges[i] * std::sin(angle);
                point.z = 0.0; // 雷达数据为2D，因此 z 轴为 0

                last_lidar_cloud_->points.push_back(point);
            }
        }

        // 填充点云头信息
        last_lidar_cloud_->width = last_lidar_cloud_->points.size();
        last_lidar_cloud_->height = 1;
        last_lidar_cloud_->is_dense = true;

        // // 将 PCL 点云转换为 ROS 消息
        // sensor_msgs::msg::PointCloud2 output;
        // pcl::toROSMsg(*cloud, output);
        // output.header.frame_id = "pcd_map";
        // output.header.stamp = msg->header.stamp;

        // // 发布点云
        // point_cloud_publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_lidar_cloud_;
    int lidar_frame_ = 0;
};

#endif