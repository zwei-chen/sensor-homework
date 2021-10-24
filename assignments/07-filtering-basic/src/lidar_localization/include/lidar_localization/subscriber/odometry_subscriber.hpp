/*
 * @Description: 订阅odometry数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {
/**
 * 里程计数据的接受者模块
 */ 
class OdometrySubscriber {
  public:
    /**
     * @brief 构造函数
     * @param nh ros句柄
     * @param topic_name 监听的话题名字
     * @param buff_size 缓冲区大小
     */ 
    OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    /**
     * @brief 默认构造函数
     */ 
    OdometrySubscriber() = default;

    /**
     * @brief 在deque_pose_data位置尾部插入new_pose_data_所有数据，并清空new_pose_data_
     * @param deque_pose_data 在里程计里存放位置数据信息的队列
     */ 
    void ParseData(std::deque<PoseData>& deque_pose_data);

  private:
    /**
     * @brief 对接收到的ros类型OdometryConstPtr数据转化为文件内定义的PoseData数据类型，并添加到new_pose_data_队列中
     * @param odom_msg_ptr 监听到的位置数据
     */ 
    void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

  private:
    ros::NodeHandle nh_;                  // ros句柄
    ros::Subscriber subscriber_;          // ros接收者
    std::deque<PoseData> new_pose_data_;  // 存放位置数据信息的队列

    std::mutex buff_mutex_;               // 锁，防止多线程数据读取冲突
};
}
#endif