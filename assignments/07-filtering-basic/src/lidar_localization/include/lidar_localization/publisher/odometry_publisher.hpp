/*
 * @Description: odometry 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
/**
 * 里程计数据发布
 */ 
class OdometryPublisher {
  public:
    /**
     * @brief 构造函数，初始化里程计数据发布者
     * @param nh ros句柄
     * @param topic_name 话题名称
     * @param base_frame_id 基坐标系
     * @param child_frame_id 子坐标系
     * @param buff_size 缓冲区长度
     */ 
    OdometryPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);

    /**
     * @brief 默认构造函数
     */    
    OdometryPublisher() = default;

    /**
     * @brief 以指定时间，发布里程计数据
     * @param transform_matrix 里程计数据
     * @param time 时间
     */ 
    void Publish(const Eigen::Matrix4f& transform_matrix, double time);

    /**
     * @brief 以当前ros时间，发布里程计数据
     * @param transform_matrix 里程计数据
     */    
    void Publish(const Eigen::Matrix4f& transform_matrix);

    /**
     * @brief 以指定时间，VelocityData类型速度数据，发布里程计数据
     * @param transform_matrix 里程计数据
     * @param velocity_data 速度数据
     * @param time 时间
     */ 
    void Publish(const Eigen::Matrix4f& transform_matrix, const VelocityData &velocity_data, double time);

    /**
     * @brief 以当前ros时间，VelocityData类型速度数据，发布里程计数据
     * @param transform_matrix 里程计数据
     * @param velocity_data 速度数据
     */ 
    void Publish(const Eigen::Matrix4f& transform_matrix, const VelocityData &velocity_data);

    /**
     * @brief 以指定时间，Vector3f类型速度数据，发布里程计数据
     * @param transform_matrix 里程计数据
     * @param vel 速度数据
     * @param time 时间
     */ 
    void Publish(const Eigen::Matrix4f& transform_matrix, const Eigen::Vector3f& vel, double time);

    /**
     * @brief 以当前ros时间，Vector3f类型速度数据，发布里程计数据
     * @param transform_matrix 里程计数据
     * @param vel 速度数据
     */ 
    void Publish(const Eigen::Matrix4f& transform_matrix, const Eigen::Vector3f& vel);

    /**
     * @brief 查看是否有节点订阅所需话题
     * @return 有节点订阅，则返回true
     */    
    bool HasSubscribers();

  private:
    /**
     * @brief 发布里程计数据
     * @param transform_matrix 位姿矩阵
     * @param velocity_data 速度
     * @param time 时间
     */  
    void PublishData(
      const Eigen::Matrix4f& transform_matrix, 
      const VelocityData &velocity_data, 
      ros::Time time
    );

  private:
    ros::NodeHandle nh_;          // 句柄
    ros::Publisher publisher_;    // 发布者

    VelocityData velocity_data_;  // 速度数据
    nav_msgs::Odometry odometry_; // 里程计数据
};
}
#endif