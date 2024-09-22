#ifndef ODOM_SUB_HPP_
#define ODOM_SUB_HPP_
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

class OdomSubscriber : public rclcpp::Node
{
public:
    OdomSubscriber() : Node("odom_subscriber")
    {
        // 创建订阅者，订阅 /odom 话题，队列大小为10
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&OdomSubscriber::odom_callback, this, std::placeholders::_1));
        log_file_.open("odom_log.txt", std::ios::out | std::ios::app);
    }

public:
    nav_msgs::msg::Odometry GetOdomData() const { return last_odom_; }
    bool HasOdomData() const
    {
        return has_odom_data_; // 判断是否有里程计数据
    }
    double GetOdomTranslate() { return delta_xy_; }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(950), [this, msg]()
                                         {
        last_odom_ = *msg;
        if (!has_odom_data_) {
            prev_odom_ = *msg;
            has_odom_data_ = true;
            return;
        } });

        // 计算位移增量
        double delta_x = msg->pose.pose.position.x - prev_odom_.pose.pose.position.x;
        double delta_y = msg->pose.pose.position.y - prev_odom_.pose.pose.position.y;
        double delta_z = msg->pose.pose.position.z - prev_odom_.pose.pose.position.z;

        // 计算二维平面上的位移增量（忽略 z 轴）
        delta_xy_ = std::sqrt(delta_x * delta_x + delta_y * delta_y);

        // 更新前一帧的 odom 数据
        prev_odom_ = *msg;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    mutable std::ofstream log_file_; // 可写的文件流对象

private:
    nav_msgs::msg::Odometry last_odom_;
    bool has_odom_data_ = false;
    nav_msgs::msg::Odometry prev_odom_; // 存储前一帧的 odom 数据
    double delta_xy_;
    rclcpp::TimerBase::SharedPtr timer_;
};
#endif