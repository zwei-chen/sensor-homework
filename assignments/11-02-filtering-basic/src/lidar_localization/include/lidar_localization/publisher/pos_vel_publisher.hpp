/*
 * @Description: synced PosVelData publisher
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_POS_VEL_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_POS_VEL_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>

#include <ros/ros.h>

#include "lidar_localization/sensor_data/pos_vel_data.hpp"

#include "lidar_localization/PosVel.h"

namespace lidar_localization {

/**
 * 同步后位置速度数据的发布者
 */ 
class PosVelPublisher {
  public:
    /**
     * @brief 构造函数
     *        构建同步后位置和速度数据的发布者
     * @param nh 句柄
     * @param topic_name 话题名字
     * @param base_frame_id 基坐标系
     * @param child_frame_id 子坐标系
     * @param buff_size 缓冲区长度
     */    
    PosVelPublisher(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      std::string base_frame_id,
      std::string child_frame_id,
      int buff_size
    );

    /**
     * @brief 默认构造函数
     */    
    PosVelPublisher() = default;

    /**
     * @brief 以指定时间，发布位置和速度数据
     * @param pos_vel_data 位置和速度数据
     * @param time 时间
     */ 
    void Publish(const PosVelData &pos_vel_data, const double &time);

    /**
     * @brief 以当前ros时间，发布位置和速度数据
     * @param pos_vel_data 位置和速度数据
     */ 
    void Publish(const PosVelData &pos_vel_data);

    /**
     * @brief 查看是否有节点订阅所需话题
     * @return 有节点订阅，则返回true
     */    
    bool HasSubscribers();

  private:
    /**
     * @brief 发布位置和速度数据
     * @param pos_vel_data 位置和速度数据
     * @param time 时间
     */  
    void PublishData(
      const PosVelData &pos_vel_data, 
      ros::Time time
    );

  private:  
    ros::NodeHandle nh_;          // 句柄
    ros::Publisher publisher_;    // 发布者
    PosVel pos_vel_msg_;          // 位置和速度数据
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_PUBLISHER_POS_VEL_PUBLISHER_HPP_