/*
 * @Description: Subscribe to PosVel messages
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "lidar_localization/PosVel.h"
#include "lidar_localization/sensor_data/pos_vel_data.hpp"

namespace lidar_localization {
/**
 * 速度位置接收模块，并解析数据
 */ 
class PosVelSubscriber {
  public:
    /**
     * @brief 构造函数
     * @param nh ros句柄
     * @param topic_name 监听的话题名字
     * @param buff_size 缓冲区大小
     */ 
    PosVelSubscriber(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      size_t buff_size
    );

    /**
     * @brief 默认构造函数
     */ 
    PosVelSubscriber() = default;

    /**
     * @brief 在pos_vel_data_buff位置尾部插入new_pos_vel_data_所有数据，并清空new_pos_vel_data_
     * @param pos_vel_data_buff 在里程计里存放点云数据信息的队列
     */ 
    void ParseData(std::deque<PosVelData>& pos_vel_data_buff);

  private:
    /**
     * @brief 对接收到的ros类型PosVelConstPtr数据转化为文件内定义的PosVelData数据类型，并添加到new_pos_vel_data_队列中
     * @param cloud_msg_ptr 监听到的点云数据
     */ 
    void msg_callback(const PosVelConstPtr& pos_vel_msg_ptr);

  private:  
    ros::NodeHandle nh_;                        // ros句柄
    ros::Subscriber subscriber_;                // ros接收者
    std::deque<PosVelData> new_pos_vel_data_;   // 存放速度位置数据信息的队列

    std::mutex buff_mutex_;                     // 锁，防止多线程数据读取冲突
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_