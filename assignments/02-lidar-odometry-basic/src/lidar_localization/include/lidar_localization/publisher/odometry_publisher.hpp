/*
 * @Description: odometry publisher
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace lidar_localization {
/**
 * 里程计信息发布模块 
 */
class OdometryPublisher {
  public:
    /**
     * @brief 构造函数，初始化
     *        里程计信息的发布者
     *        里程计信息的base_frame_id和child_frame_id
     * @param nh ros用句柄
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
     * @brief 将Eigen::Matrix4f类型的transform_matrix转化为里程计信息并发布
     * @param transform_matrix Eigen::Matrix4f类型的里程计信息
     */ 
    void Publish(const Eigen::Matrix4f& transform_matrix);

  private:
    ros::NodeHandle nh_;          // ros句柄
    ros::Publisher publisher_;    // ros发布者
    nav_msgs::Odometry odometry_; // ros用里程计信息
};
}
#endif