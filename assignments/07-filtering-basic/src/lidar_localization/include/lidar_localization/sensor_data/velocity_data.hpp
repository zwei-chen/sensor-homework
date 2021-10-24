/*
 * @Description: velocity 数据
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

namespace lidar_localization {
class VelocityData {
  public:
    // 线速度
    struct LinearVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    // 角速度
    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    double time = 0.0;                  // 时间
    LinearVelocity linear_velocity;     // 线速度
    AngularVelocity angular_velocity;   // 角速度
  
  public:
    /**
     * @brief 速度数据的时间同步
     *        使用线性插值的方法来对线速度和角速度进行求解
     * @param UnsyncedData  未同步的速度数据
     * @param SyncedData  已经同步的速度数据
     * @param sync_time   同步的时间
     * @return 是否成功完成时间同步
     */     
    static bool SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time);

    /**
     * @brief 从雷达与惯导的位姿矩阵更新线速度和角速度的坐标系
     *        最后获得在雷达坐标系下的速度与加速度
     * @param transform_matrix 雷达与惯导的位姿矩阵
     */    
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);

    /**
     * @brief 将北东地坐标系转化为东北天坐标系
     */    
    void NED2ENU(void);
};
}
#endif