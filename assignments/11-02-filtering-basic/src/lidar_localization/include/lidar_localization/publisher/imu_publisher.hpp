/*
 * @Description: 在ros中发布IMU数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {
class IMUPublisher {
  public:
    /**
     * @brief 构造函数，初始化imu数据的发布者
     * @param nh ros句柄
     * @param topic_name 话题名称
     * @param frame_id 坐标系
     * @param buff_size 缓冲区长度
     */ 
    IMUPublisher(
      ros::NodeHandle& nh,
      std::string topic_name,
      std::string frame_id,
      size_t buff_size
    );

    /**
     * @brief 默认构造函数
     */ 
    IMUPublisher() = default;

    /**
     * @brief 以指定时间，发布imu数据
     * @param imu_data imu数据
     * @param time 时间
     */ 
    void Publish(const IMUData &imu_data, double time);

    /**
     * @brief 以当前ros时间，发布imu数据
     * @param imu_data imu数据
     */ 
    void Publish(const IMUData &imu_data);

    /**
     * @brief 查看是否有节点订阅所需话题
     * @return 有节点订阅，则返回true
     */    
    bool HasSubscribers(void);

  private:
    /**
     * @brief 发布imu数据
     * @param imu_data imu数据
     * @param time 时间
     */ 
    void PublishData(const IMUData &imu_data, ros::Time time);

    ros::NodeHandle nh_;          // ros句柄
    ros::Publisher publisher_;    // ros发布者
    std::string frame_id_;        // 将要发布数据的坐标系

    sensor_msgs::Imu imu_;        // 发布的imu数据
};
} 
#endif