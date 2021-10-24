/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>

namespace lidar_localization {
/**
 * IMU数据类型
 */ 
class IMUData {
  public:
    // 线加速度
    struct LinearAcceleration {
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

    // 航向角
    // 一个四元数类型
    class Orientation {
      public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
      
      public:
        /**
         * @brief 归一化
         */ 
        void Normlize() {
          double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
          x /= norm;
          y /= norm;
          z /= norm;
          w /= norm;
        }
    };

    double time = 0.0;                          // 时间
    LinearAcceleration linear_acceleration;     // 线加速度
    AngularVelocity angular_velocity;           // 角速度
    Orientation orientation;                    // 航向角
  
  public:
    /**
     * @brief 把四元数转换成旋转矩阵送出去
     * @return 转换后的旋转矩阵
     */     
    Eigen::Matrix3f GetOrientationMatrix() const;

    /**
     * @brief imu数据的时间同步
     *        使用线性插值的方法来对线加速度、角速度和航向角进行求解
     * @param UnsyncedData  未同步的imu数据
     * @param SyncedData  已经同步的imu数据
     * @param sync_time   同步的时间
     * @return 是否成功完成时间同步
     */   
    static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);
};
}
#endif