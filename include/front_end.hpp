
#ifndef FRONT_END_HPP_
#define FRONT_END_HPP_
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

class FrontEnd
{
 public:
 FrontEnd();
 void FilterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
                      float leaf_size) ;
 void ScanMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
                const Eigen::Matrix4d& initial_transform,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud,
                Eigen::Matrix4d& final_transform);

 private:

};

#endif