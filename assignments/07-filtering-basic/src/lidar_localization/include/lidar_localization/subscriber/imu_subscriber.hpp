/*
 * @Description: 订阅imu数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {
/**
 * imu数据接收处理模块
 */ 
class IMUSubscriber {
  public:
    /**
     * @brief 构造函数，初始化
     * @param nh ros句柄
     * @param topic_name 话题名称
     * @param buff_size 缓冲区长度
     */  
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    
    /**
     * @brief 默认构造函数
     */    
    IMUSubscriber() = default;

    /**
     * @brief 在deque_imu_data位置尾部插入new_imu_data_所有数据，并清空new_imu_data_
     * @param deque_imu_data 在里程计里存放imu数据信息的队列
     */ 
    void ParseData(std::deque<IMUData>& deque_imu_data);

  private:
    /**
     * @brief 对接收到的ros类型imu数据转化为文件内定义imu数据类型，并添加到imu_msg_ptr队列中
     * @param imu_msg_ptr ros类型的imu数据
     */    
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

  private:
    ros::NodeHandle nh_;                // ros句柄
    ros::Subscriber subscriber_;        // ros接收者
    std::deque<IMUData> new_imu_data_;  // 存放imu数据的队列

    std::mutex buff_mutex_;             // 锁
};
}
#endif