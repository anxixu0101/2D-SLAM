#include "include/time_sync.hpp"

OdomLidarSync::OdomLidarSync() : Node("odom_lidar_sync") {
    // 订阅 Odom 数据
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OdomLidarSync::odomCallback, this, std::placeholders::_1));
    
    // 订阅 Lidar 数据
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", 10, std::bind(&OdomLidarSync::lidarCallback, this, std::placeholders::_1));
    log_file_.open("odom_data.txt", std::ios::out | std::ios::app);
}

// Odom 数据回调函数
void OdomLidarSync::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 将 Odom 数据添加到缓存
    odom_buffer_.push_back(msg);

    // 仅保留一定数量的缓存，避免内存过大
    if (odom_buffer_.size() > 100) {
        odom_buffer_.pop_front();
    }
}

// Lidar 数据回调函数
void OdomLidarSync::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // 将 Lidar 数据添加到缓存
    lidar_buffer_.push_back(msg);

    // 仅保留一定数量的缓存，避免内存过大
    if (lidar_buffer_.size() > 100) {
        lidar_buffer_.pop_front();
    }

    // 每次接收到新的 Lidar 数据时尝试同步
    synchronizeData();
}

// 手动同步 Odom 和 Lidar 数据
void OdomLidarSync::synchronizeData() {
    // 确保缓存中有数据
    if (odom_buffer_.empty() || lidar_buffer_.empty()) {
        return;  // 如果任一缓存为空，无法同步
    }

    auto lidar_msg = lidar_buffer_.front();
    rclcpp::Time lidar_time = lidar_msg->header.stamp;

    // 查找与 Lidar 时间戳最接近的 Odom 消息
    nav_msgs::msg::Odometry::SharedPtr closest_odom = nullptr;
    rclcpp::Time closest_odom_time;
    double min_time_diff = std::numeric_limits<double>::max();

    for (const auto& odom_msg : odom_buffer_) {
        rclcpp::Time odom_time = odom_msg->header.stamp;
        double time_diff = std::abs((odom_time - lidar_time).seconds());

        // 如果找到时间差更小的 Odom 消息，更新
        if (time_diff < min_time_diff) {
            closest_odom = odom_msg;
            closest_odom_time = odom_time;
            min_time_diff = time_diff;
        }
    }

    // 设置时间差阈值（例如 0.05 秒），避免不合理的同步
    const double time_threshold = 0.05;

    if (closest_odom && min_time_diff < time_threshold) {
        // 输出同步成功的 Odom 和 Lidar 数据
        // RCLCPP_INFO(this->get_logger(), "Synchronized Odom and Lidar:");
        // RCLCPP_INFO(this->get_logger(), "Odom timestamp: %.2f, Lidar timestamp: %.2f", closest_odom_time.seconds(), lidar_time.seconds());
       // std::cout<<"-------->> "<<closest_odom->pose.pose.position.x<<" "<<closest_odom->pose.pose.position.y<<std::endl;
        log_file_<<closest_odom->pose.pose.position.x<<" "<<closest_odom->pose.pose.position.y<<std::endl;
        // 同步成功后，移除相应的 Odom 和 Lidar 消息，避免重复处理
        odom_buffer_.erase(std::remove(odom_buffer_.begin(), odom_buffer_.end(), closest_odom), odom_buffer_.end());
        lidar_buffer_.pop_front();
    } else {
        // 如果找不到合适的同步消息，移除 Lidar 消息
        lidar_buffer_.pop_front();
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
