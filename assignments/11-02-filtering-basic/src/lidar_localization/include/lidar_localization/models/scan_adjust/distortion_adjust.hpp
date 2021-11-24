/*
 * @Description: 点云畸变补偿
 * @Author: Ren Qian
 * @Date: 2020-02-25 14:38:12
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "lidar_localization/sensor_data/velocity_data.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class DistortionAdjust {
  public:
    /**
     * @brief 读取畸变校正所需的数据
     *        依次读取扫描时间、线速度和角速度
     * @param scan_period lidar完成一次扫描使用的时间
     */
    void SetMotionInfo(float scan_period, VelocityData velocity_data);

    /**
     * @brief 对点云进行校正处理
     * @param input_cloud_ptr 校正前的点云
     * @param output_cloud_ptr 校正后的点云
     * @return 只会返回ture，可视作是否完成的标识
     */    
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

  private:
    /**
     * @brief 根据角加速度计算每一个轴的旋转向量，进而计算整体的旋转矩阵
     *        内联函数。这样可以解决一些频繁调用的函数大量消耗栈空间（栈内存）的问题
     * @param float real_time 当前点扫描的具体时间
     * @return 旋转矩阵
     */    
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

  private:
    float scan_period_;             // 扫描时间
    Eigen::Vector3f velocity_;      // lidar当前线速度
    Eigen::Vector3f angular_rate_;  // lidar当前角速度
};
} // namespace lidar_slam
#endif