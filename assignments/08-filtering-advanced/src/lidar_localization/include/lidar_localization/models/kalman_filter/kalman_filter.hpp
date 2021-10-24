/*
 * @Description: Kalman Filter interface.
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_HPP_

#include <yaml-cpp/yaml.h>

#include <deque>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {

/**
 * 卡尔曼滤波基类
 */
class KalmanFilter {
public:
  /**
   * @class MeasurementType 测量的数据类型
   * @brief enum for observation type
   */
  enum MeasurementType {
    POSE = 0,
    POSE_VEL,
    POSI,
    POSI_VEL,
    POSI_MAG,
    POSI_VEL_MAG,
    NUM_TYPES
  };

  /**
   * @class Measurement 测量数据
   * @brief Kalman filter measurement data
   *        卡尔曼滤波测量数据
   */
  struct Measurement {
    // timestamp: 时间
    double time;
    // a. pose observation, lidar/visual frontend: 位姿，由雷达和视觉前端提供
    Eigen::Matrix4d T_nb;
    // b. body frame velocity observation, odometer: 速度
    Eigen::Vector3d v_b;
    // c. body frame angular velocity, needed by motion constraint: 加速度
    Eigen::Vector3d w_b;
    // d. magnetometer:  磁力计
    Eigen::Vector3d B_b;
  };

  /**
   * @class Cov
   * @brief Kalman filter process covariance data
   */
  struct Cov {
    struct {
      double x;
      double y;
      double z;
    } pos;
    struct {
      double x;
      double y;
      double z;
    } vel;
    // here quaternion is used for orientation representation:
    struct {
      double w;
      double x;
      double y;
      double z;
    } ori;
    struct {
      double x;
      double y;
      double z;
    } gyro_bias;
    struct {
      double x;
      double y;
      double z;
    } accel_bias;
  };

  /**
   * @brief  init filter
   * @param  imu_data, input IMU measurements
   * @return true if success false otherwise
   */
  virtual void Init(const Eigen::Vector3d &vel, const IMUData &imu_data) = 0;

  /**
   * @brief  update state & covariance estimation, Kalman prediction
   * @param  imu_data, input IMU measurements
   * @return true if success false otherwise
   */
  virtual bool Update(const IMUData &imu_data) = 0;

  /**
   * @brief  correct state & covariance estimation, Kalman correction
   * @param  measurement_type, input measurement type
   * @param  measurement, input measurement
   * @return void
   */
  virtual bool Correct(const IMUData &imu_data,
                       const MeasurementType &measurement_type,
                       const Measurement &measurement) = 0;

  /**
   * @brief  get filter time
   * @return filter time as double
   */
  double GetTime(void) const { return time_; }

  /**
   * @brief  get odometry estimation
   * @param  pose, output pose
   * @param  vel, output vel
   * @return void
   */
  virtual void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel) = 0;

  /**
   * @brief  get covariance estimation
   * @param  cov, output covariance
   * @return void
   */
  virtual void GetCovariance(Cov &cov) = 0;

  /**
   * @brief  update observability analysis
   * @param  time, measurement time
   * @param  measurement_type, measurement type
   * @return void
   */
  virtual void
  UpdateObservabilityAnalysis(const double &time,
                              const MeasurementType &measurement_type) = 0;

  /**
   * @brief  save observability analysis to persistent storage
   * @param  measurement_type, measurement type
   * @return void
   */
  virtual bool
  SaveObservabilityAnalysis(const MeasurementType &measurement_type) = 0;

protected:
  KalmanFilter() {}

  static void AnalyzeQ(const int DIM_STATE, const double &time,
                       const Eigen::MatrixXd &Q, const Eigen::VectorXd &Y,
                       std::vector<std::vector<double>> &data);

  static void WriteAsCSV(const int DIM_STATE,
                         const std::vector<std::vector<double>> &data,
                         const std::string filename);

  // time:
  double time_;   // 上一帧数据的时间戳

  // data buff:
  std::deque<IMUData> imu_data_buff_;       // imu数据的缓存区

  // earth constants: 地球常量
  Eigen::Vector3d g_;   // 重力加速度常量
  Eigen::Vector3d w_;
  Eigen::Vector3d b_;

  // observability analysis:
  struct {
    std::vector<double> time_;
    std::vector<Eigen::MatrixXd> Q_;
    std::vector<Eigen::VectorXd> Y_;
  } observability;

  // hyper-params:
  // a. earth constants: 地球常量
  struct {
    double GRAVITY_MAGNITUDE; // 重力加速度常量
    double ROTATION_SPEED;
    double LATITUDE;          // 纬度
    double LONGITUDE;
    struct {
      double B_E;             // 东
      double B_N;             // 北
      double B_U;             // 天
    } MAG;
  } EARTH;
  // b. prior state covariance, process & measurement noise:
  struct {
    struct {
      double POSI;      // p
      double VEL;       // v
      double ORI;       // theta
      double EPSILON;   // ba
      double DELTA;     // bw
    } PRIOR;   // 先验状态的方差
    struct {
      double GYRO;        // w
      double ACCEL;       // a
      double BIAS_ACCEL;  // ba
      double BIAS_GYRO;   // bw
    } PROCESS;  // 过程噪声的矩阵
    struct {
      struct {
        double POSI;  // delta p
        double ORI;   // delta theta
      } POSE;
      double POSI;    //
      double VEL;     // delta v
      double ORI;
      double MAG;
    } MEASUREMENT;  // 测量噪声
  } COV;  // 协方差数据
  // c. motion constraint:
  struct {
    bool ACTIVATED;
    double W_B_THRESH;
  } MOTION_CONSTRAINT;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_HPP_