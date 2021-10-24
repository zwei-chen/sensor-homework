/*
 * @Description: TF broadcaster
 * @Author: Ge Yao
 * @Date: 2021-04-03
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
 * TF发布者模块
 * child_frame_id 到 base_frame_id 的转换矩阵
 */ 
class TFBroadCaster {
  public:

    /**
     * @brief 构造函数，主要是对变量transform_初始化base_frame_id和child_frame_id
     * @param frame_id  map坐标系
     * @param child_frame_id 子坐标系
     */ 
    TFBroadCaster(std::string frame_id, std::string child_frame_id);

    /**
     * @brief 默认构造函数
     */ 
    TFBroadCaster() = default;

    /**
     * @brief  使用broadcaster_发布transform_类型数据
     * @param pose  位姿
     * @param time  时间
     */ 
    void SendTransform(Eigen::Matrix4f pose, double time);
  protected:
    tf::StampedTransform transform_;        // 需要发布的数据，包含时间和位姿
    tf::TransformBroadcaster broadcaster_;  // 发布数据的类型
};

}

#endif // namespace lidar_localization