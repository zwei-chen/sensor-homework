/*
 * @Description: tf监听模块
 * @Author: Ren Qian
 * @Date: 2020-02-06 16:01:21
 */
#ifndef LIDAR_LOCALIZATION_TF_LISTENER_HPP_
#define LIDAR_LOCALIZATION_TF_LISTENER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace lidar_localization {
/**
 * TF接收者模块
 * child_frame_id 到 base_frame_id 的转换矩阵
 */ 
class TFListener {
  public:
    /**
     * @brief 构造函数，主要是初始化base_frame_id和child_frame_id
     * @param nh ros句柄
     * @param base_frame_id 基坐标系
     * @param child_frame_id 子坐标系
     */ 
    TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);

    /**
     * @brief 默认构造函数
     */ 
    TFListener() = default;

    /**
     * @brief 获取child_frame_id_到base_frame_id_的位姿矩阵，并转化给transform_matrix
     * @param transform_matrix Eigen::Matrix4f类型的位姿矩阵
     */ 
    bool LookupData(Eigen::Matrix4f& transform_matrix);
  
  private:
    /**
     * @brief 将transform的tf::StampedTransform数据类型转化为transform_matrix的Eigen::Matrix4f数据类型
     * @param transform tf::StampedTransform的位姿矩阵
     * @param transform_matrix Eigen::Matrix4f类型的位姿矩阵
     */ 
    bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

  private:
    ros::NodeHandle nh_;                // ros用句柄
    tf::TransformListener listener_;    // 该模块监听的数据类型
    std::string base_frame_id_;         // 基坐标系
    std::string child_frame_id_;        // 子坐标系
};
}

#endif