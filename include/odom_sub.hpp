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
    
    nav_msgs::msg::Odometry GetOdomData() const{ return last_odom_;}
    bool HasOdomData() const {
        return has_odom_data_;  // 判断是否有里程计数据
    }
private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {

        last_odom_ = *msg;
        has_odom_data_ = true;
        // // 获取机器人在坐标系中的位置
        // auto position = msg->pose.pose.position;
        // auto orientation = msg->pose.pose.orientation;

        // // 打印位置信息
       //  RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, z=%.2f", position.x, position.y, position.z);

        // // 打印方向信息（四元数表示）
        // RCLCPP_INFO(this->get_logger(), "Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", 
        //             orientation.x, orientation.y, orientation.z, orientation.w);
        // if (log_file_.is_open()) {
        //     log_file_ << position.x << " " << position.y  << std::endl;
        //    // log_file_ << "Orientation: x=" << orientation.x << ", y=" << orientation.y << ", z=" << orientation.z << ", w=" << orientation.w << std::endl;
        // }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    mutable std::ofstream log_file_;  // 可写的文件流对象

private:
    nav_msgs::msg::Odometry last_odom_; 
    bool has_odom_data_ = false;

};
#endif