#include "include/time_sync.hpp"

OdomLidarSync::OdomLidarSync() : Node("odom_lidar_sync")
{
    // 订阅 Odom 数据
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OdomLidarSync::odomCallback, this, std::placeholders::_1));

    // 订阅 Lidar 数据
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", 10, std::bind(&OdomLidarSync::lidarCallback, this, std::placeholders::_1));
    map_builder_ = std::make_shared<MapBuilder>();
    last_lidar_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

// Odom 数据回调函数
void OdomLidarSync::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 添加 odom 数据到缓存
    odom_buffer_.push_back(msg);
    // 尝试进行时间同步
    synchronizeData();
}

// Lidar 数据回调函数
void OdomLidarSync::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    lidar_frame_++;
    // 添加 Lidar 数据到缓存
    lidar_buffer_.push_back(msg);
    // 尝试进行时间同步
    synchronizeData();
}

// 手动同步 Odom 和 Lidar 数据
void OdomLidarSync::synchronizeData()
{
    // 检查缓存中是否有数据
    if (odom_buffer_.empty() || lidar_buffer_.empty())
    {
        return; // 如果任一缓存为空，无法同步
    }

    // 获取最近的 Odom 和 Lidar 数据
    auto odom_msg = odom_buffer_.front();
    auto lidar_msg = lidar_buffer_.front();

    // 检查 Odom 和 Lidar 数据的时间戳差异
    rclcpp::Time odom_time = odom_msg->header.stamp;
    rclcpp::Time lidar_time = lidar_msg->header.stamp;

    // 时间差异阈值（例如 0.05 秒）
    const double time_threshold = 0.05;

    // 比较时间戳
    if (std::abs((odom_time - lidar_time).seconds()) < time_threshold)
    {
        // 时间戳足够接近，执行同步后的处理
        // RCLCPP_INFO(this->get_logger(), "Synchronized Odom and Lidar:");
        // RCLCPP_INFO(this->get_logger(), "Odom timestamp: %.2f, Lidar timestamp: %.2f", odom_time.seconds(), lidar_time.seconds());
        std::cout << "-------------->>" << lidar_frame_ << std::endl;
        // 将 LaserScan 数据转换为 PCL 格式并处理
       
        convertLaserScanToPointCloud(lidar_msg);

        map_builder_->buildMap(odom_msg, last_lidar_cloud_, lidar_frame_);

        // RCLCPP_INFO(this->get_logger(), "Converted Lidar to PCL format with %lu points.", cloud_ptr->points.size());

        // 移除已经同步的消息
        odom_buffer_.pop_front();
        lidar_buffer_.pop_front();
    }
    else
    {
        // 如果时间差异较大，移除较早的消息
        if (odom_time < lidar_time)
        {
            odom_buffer_.pop_front();
        }
        else
        {
            lidar_buffer_.pop_front();
        }
    }
}

// 将 LaserScan 转换为 PCL 点云
void OdomLidarSync::convertLaserScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr &msg)
{
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
}
