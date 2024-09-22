// map_builder.cpp
#include "include/map_builder.hpp"

MapBuilder::MapBuilder(std::shared_ptr<OdomSubscriber> odom_sub, std::shared_ptr<LidarListener> lidar_sub)
    : Node("map_builder"), odom_sub_(odom_sub), lidar_sub_(lidar_sub)
{
    map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_point_cloud", 10);
    front_end_ = std::make_shared<FrontEnd>();
}

void MapBuilder::buildMap()
{
    auto odom = odom_sub_->GetOdomData();
    last_odom_stamp_ = odom.header.stamp;
    auto lidar_cloud = lidar_sub_->GetLidarData();
    auto laser_cloud = lidar_sub_->GetLaserData();
    last_lidar_stamp_ = laser_cloud.header.stamp;
    if (lidar_cloud->points.size() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "sensor size is 0");
        return;
    }

    // 使用 odom 位姿信息构造变换矩阵
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation(0, 3) = odom.pose.pose.position.x;
    transformation(1, 3) = odom.pose.pose.position.y;
    transformation(2, 3) = odom.pose.pose.position.z;
    // std::cout<<transformation(0, 3)<<" "<<transformation(1, 3)<<std::endl;
    Eigen::Quaternionf rotation(
        odom.pose.pose.orientation.w,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z);
    transformation.block<3, 3>(0, 0) = rotation.toRotationMatrix().transpose();

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4d final_transform = Eigen::Matrix4d::Identity();
    front_end_->FilterPointCloud(lidar_cloud, lidar_cloud, 0.05);

  

    
    if (lidar_sub_->GetLidarFrame() == 1)
    {
        RCLCPP_INFO(this->get_logger(), "Map initial success!");
        *map_cloud_ = *lidar_cloud;
    }
    else
    {
        if (last_odom_stamp_.get_clock_type() != last_lidar_stamp_.get_clock_type())
        {
            RCLCPP_ERROR(this->get_logger(), "Time sources are different! Cannot compare.");
            return;
        }
        // 计算时间差
        double time_diff = std::abs((last_odom_stamp_ - last_lidar_stamp_).seconds());

        if (time_diff <= 0.2&&odom_sub_->HasOdomData())
        {
            //front_end_->FilterPointCloud(lidar_cloud,lidar_cloud,0.05f);
            front_end_->ScanMatch(lidar_cloud, map_cloud_, transformation.cast<double>(),
                                  aligned_cloud, final_transform);

             pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            // //  将激光雷达点云转换到全局坐标系

            robot_pose_ = transformation.cast<double>() * final_transform;
            pcl::transformPointCloud(*lidar_cloud, *transformed_cloud, robot_pose_);
            *map_cloud_ += *aligned_cloud;
            //front_end_->FilterPointCloud(map_cloud_, map_cloud_, 0.05f);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Time difference too large: %.3f seconds. Ignoring...", time_diff);
        }
    }
    if (map_cloud_ != nullptr)
    {
        publishMap();
    }
    }
    // RCLCPP_INFO(this->get_logger(), "Map updated with new point cloud data.");


void MapBuilder::publishMap()
{
    // 将 PCL 点云转换为 ROS2 消息格式 (PointCloud2)
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*map_cloud_, output);

    // 设置消息头
    output.header.frame_id = "map"; // 固定帧 "map"（根据需要修改）
    output.header.stamp = this->get_clock()->now();

    // 发布点云地图
    map_publisher_->publish(output);
}