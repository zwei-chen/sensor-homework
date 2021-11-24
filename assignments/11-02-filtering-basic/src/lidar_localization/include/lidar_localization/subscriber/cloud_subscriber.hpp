/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
/**
 * 点云信息接收模块，并解析数据
 */ 
class CloudSubscriber {
  public:
    /**
     * @brief 构造函数
     * @param nh ros句柄
     * @param topic_name 监听的话题名字
     * @param buff_size 缓冲区大小
     */ 
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    /**
     * @brief 默认构造函数
     */ 
    CloudSubscriber() = default;

    /**
     * @brief 在deque_cloud_data位置尾部插入new_cloud_data_所有数据，并清空new_cloud_data_
     * @param deque_cloud_data 在里程计里存放点云数据信息的队列
     */ 
    void ParseData(std::deque<CloudData>& deque_cloud_data);

  private:
    /**
     * @brief 对接收到的ros类型点云数据转化为文件内定义的点云数据类型，并添加到new_cloud_data_队列中
     * @param cloud_msg_ptr 监听到的点云数据
     */ 
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;                      // ros句柄
    ros::Subscriber subscriber_;              // ros接收者
    std::deque<CloudData> new_cloud_data_;    // 存放点云数据信息的队列

    std::mutex buff_mutex_;                   // 锁，防止多线程数据读取冲突
};
}

#endif