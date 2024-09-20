
#ifndef FRONT_END_HPP_
#define FRONT_END_HPP_
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
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
 pcl::PointCloud<pcl::PointXYZ>::Ptr ScanMatch(pcl::PointCloud<pcl::PointXYZ>::Ptr src,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                               Eigen::Matrix4d predict);

 private:

};

#endif