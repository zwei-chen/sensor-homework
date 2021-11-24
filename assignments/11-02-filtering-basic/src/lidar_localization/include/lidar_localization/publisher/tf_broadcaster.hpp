/*
 * @Description: 发布tf的类
 * @Author: Ren Qian
 * @Date: 2020-03-05 15:23:26
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_TF_BROADCASTER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_TF_BROADCASTER_HPP_

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace lidar_localization {
/**
 * 发布tf变换的模块
 */ 
class TFBroadCaster {
  public:
    /**
     * @brief 构造函数，初始化tf变换数据类型的基坐标系和子坐标系
     * @param frame_id 基坐标系
     * @param child_frame_id 子坐标系
     */ 
    TFBroadCaster(std::string frame_id, std::string child_frame_id);

    /**
     * @brief 默认构造函数
     */    
    TFBroadCaster() = default;

    /**
     * @brief 初始化tf变换数据类型的时间、旋转和平移
     *        并发布
     * @param pose 位姿
     * @param time 时间
     */    
    void SendTransform(Eigen::Matrix4f pose, double time);
  protected:
    tf::StampedTransform transform_;         // tf变换的数据类型
    tf::TransformBroadcaster broadcaster_;   // tf发布类型
};
}
#endif