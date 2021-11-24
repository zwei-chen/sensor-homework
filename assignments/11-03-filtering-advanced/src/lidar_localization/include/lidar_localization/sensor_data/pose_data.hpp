/*
 * @Description: 存放处理后的IMU姿态以及GNSS位置
 * @Author: Ren Qian
 * @Date: 2020-02-27 23:10:56
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

namespace lidar_localization {

/**
 * 处理后的IMU姿态以及GNSS位置数据类型
 */ 
class PoseData {
  public:
    double time = 0.0;                                  // 时间
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(); // 位姿
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();      // 速度

  public:
    /**
     * @brief 获取旋转部分的四元数表示
     * @return 旋转部分的四元数表示
     */    
    Eigen::Quaternionf GetQuaternion();

    static bool SyncData(std::deque<PoseData>& UnsyncedData, std::deque<PoseData>& SyncedData, double sync_time);
};

}

#endif