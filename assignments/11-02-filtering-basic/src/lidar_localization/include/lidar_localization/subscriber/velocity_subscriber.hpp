/*
 * @Description: 订阅velocity数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"

#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class VelocitySubscriber {
  public:
    /**
     * @brief 构建速度数据的接受者
     * @param topic_name 话题名称
     * @param buff_size 缓存大小
     */    
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    /**
     * @brief 默认构造函数
     */ 
    VelocitySubscriber() = default;

    /**
     * @brief 在deque_velocity_data位置尾部插入new_velocity_data_所有数据，并清空new_velocity_data_
     * @param deque_velocity_data 在里程计里存放速度数据信息的队列
     */ 
    void ParseData(std::deque<VelocityData>& deque_velocity_data);

  private:
    /**
     * @brief 对接收到的ros类型速度数据转化为文件内定义的速度数据类型，并添加到new_velocity_data_队列中
     * @param twist_msg_ptr 监听到的速度数据
     */ 
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;                            // ros句柄
    ros::Subscriber subscriber_;                    // ros接受者
    std::deque<VelocityData> new_velocity_data_;    // 速度数据的队列

    std::mutex buff_mutex_;                         // 锁
};
}
#endif