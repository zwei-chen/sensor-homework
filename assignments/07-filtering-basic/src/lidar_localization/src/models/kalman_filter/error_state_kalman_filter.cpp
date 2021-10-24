/*
 * @Description: Error-State Kalman Filter for IMU-Lidar-GNSS-Odo fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include <cstdlib>
#include <limits>

#include <cmath>
#include <fstream>
#include <iostream>
#include <ostream>

// use sophus to handle so3 hat & SO3 log operations:
#include <sophus/so3.hpp>

#include "lidar_localization/models/kalman_filter/error_state_kalman_filter.hpp"

#include "lidar_localization/global_defination/global_defination.h"

#include "glog/logging.h"

namespace lidar_localization {

ErrorStateKalmanFilter::ErrorStateKalmanFilter(const YAML::Node &node) {
  //
  // parse config: 解析配置文件
  //
  // a. earth constants: 地球常量
  // 读取重力加速度常量
  EARTH.GRAVITY_MAGNITUDE = node["earth"]["gravity_magnitude"].as<double>();
  // 读取纬度
  EARTH.LATITUDE = node["earth"]["latitude"].as<double>();
  EARTH.LATITUDE *= M_PI / 180.0; // 转换坐标形式

  // b. prior state covariance: 先验状态的协方差
  COV.PRIOR.POSI = node["covariance"]["prior"]["pos"].as<double>();
  COV.PRIOR.VEL = node["covariance"]["prior"]["vel"].as<double>();
  COV.PRIOR.ORI = node["covariance"]["prior"]["ori"].as<double>();
  COV.PRIOR.EPSILON = node["covariance"]["prior"]["epsilon"].as<double>();
  COV.PRIOR.DELTA = node["covariance"]["prior"]["delta"].as<double>();

  // c. process noise: 过程噪声的协方差矩阵
  COV.PROCESS.ACCEL = node["covariance"]["process"]["accel"].as<double>();
  COV.PROCESS.GYRO = node["covariance"]["process"]["gyro"].as<double>();
  COV.PROCESS.BIAS_ACCEL =
      node["covariance"]["process"]["bias_accel"].as<double>();
  COV.PROCESS.BIAS_GYRO =
      node["covariance"]["process"]["bias_gyro"].as<double>();

  // d. measurement noise: 测量噪声的协方差矩阵
  COV.MEASUREMENT.POSE.POSI =
      node["covariance"]["measurement"]["pose"]["pos"].as<double>();
  COV.MEASUREMENT.POSE.ORI =
      node["covariance"]["measurement"]["pose"]["ori"].as<double>();
 
  // prompt: 输出对应的卡尔曼滤波器参数
  LOG(INFO) << std::endl
            << "Error-State Kalman Filter params:" << std::endl
            << "\tgravity magnitude: " << EARTH.GRAVITY_MAGNITUDE << std::endl
            << "\tlatitude: " << EARTH.LATITUDE << std::endl
            << std::endl
            << "\tprior cov. pos.: " << COV.PRIOR.POSI << std::endl
            << "\tprior cov. vel.: " << COV.PRIOR.VEL << std::endl
            << "\tprior cov. ori: " << COV.PRIOR.ORI << std::endl
            << "\tprior cov. epsilon.: " << COV.PRIOR.EPSILON << std::endl
            << "\tprior cov. delta.: " << COV.PRIOR.DELTA << std::endl
            << std::endl
            << "\tprocess noise gyro.: " << COV.PROCESS.GYRO << std::endl
            << "\tprocess noise accel.: " << COV.PROCESS.ACCEL << std::endl
            << std::endl
            << "\tmeasurement noise pose.: " << std::endl
            << "\t\tpos: " << COV.MEASUREMENT.POSE.POSI
            << ", ori.: " << COV.MEASUREMENT.POSE.ORI << std::endl
            << std::endl
            << std::endl;

  //
  // init filter: 初始化滤波器
  //
  // a. earth constants: 地球常量
  g_ = Eigen::Vector3d(0.0, 0.0, EARTH.GRAVITY_MAGNITUDE);
  // b. prior state & covariance: 重置状态和先验协方差矩阵
  // 此步对应2. Kalman初始化的a.
  ResetState();
  ResetCovariance();

  // 此步对应2. Kalman初始化的b.
  // c. process noise: 过程噪声
  Q_.block<3, 3>(kIndexNoiseAccel, kIndexNoiseAccel) = COV.PROCESS.ACCEL * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseGyro, kIndexNoiseGyro) = COV.PROCESS.GYRO * Eigen::Matrix3d::Identity();
  //Q_.block<3, 3>(kIndexNoiseBiasAccel, kIndexNoiseBiasAccel) = Eigen::Matrix3d::Zero();
  //Q_.block<3, 3>(kIndexNoiseBiasGyro, kIndexNoiseBiasGyro) = Eigen::Matrix3d::Zero();
  Q_.block<3, 3>(kIndexNoiseBiasAccel, kIndexNoiseBiasAccel) = COV.PROCESS.BIAS_ACCEL * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasGyro, kIndexNoiseBiasGyro) = COV.PROCESS.BIAS_GYRO * Eigen::Matrix3d::Identity();

  // d. measurement noise: 测量噪声
  RPose_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSE.POSI * Eigen::Matrix3d::Identity();
  RPose_.block<3, 3>(3, 3) = COV.MEASUREMENT.POSE.ORI * Eigen::Matrix3d::Identity();

  // e. process equation: 过程方程
  // 只初始化过程方程的几个常量位置
  F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) = -Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = Eigen::Matrix3d::Identity();

  // f. measurement equation: 测量方程
  GPose_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
  GPose_.block<3, 3>(3, kIndexErrorOri) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  // init soms:
  // 输入噪声的方差矩阵
  QPose_.block<kDimMeasurementPose, kDimState>(0, 0) = GPose_;
}

void ErrorStateKalmanFilter::Init(const Eigen::Vector3d &vel,
                                  const IMUData &imu_data) {
  // init odometry:
  // 初始化里程计
  Eigen::Matrix3d C_nb = imu_data.GetOrientationMatrix().cast<double>();
  // a. init C_nb using IMU estimation:
  // 用imu估计得姿态作为初始位姿
  pose_.block<3, 3>(0, 0) = C_nb;
  // b. convert flu velocity into navigation frame:
  // 将速度转换到定位坐标系下
  vel_ = C_nb * vel;

  // save init pose:
  // 设置初始位姿
  init_pose_ = pose_;

  // init IMU data buffer:
  // 初始化imu数据的缓存区
  imu_data_buff_.clear();
  imu_data_buff_.push_back(imu_data);

  // init filter time:
  // 初始化上一帧数据的时间戳
  time_ = imu_data.time;

  // set process equation in case of one step prediction & correction:
  // 设置过程方程的a和w
  Eigen::Vector3d linear_acc_init(imu_data.linear_acceleration.x,
                                  imu_data.linear_acceleration.y,
                                  imu_data.linear_acceleration.z);
  Eigen::Vector3d angular_vel_init(imu_data.angular_velocity.x,
                                   imu_data.angular_velocity.y,
                                   imu_data.angular_velocity.z);
  // covert to navigation frame:
  // 转换坐标系，课程使用的是body坐标系
  // TODO!: 遗留错误修改
  //linear_acc_init = GetUnbiasedLinearAcc(linear_acc_init, C_nb);
  linear_acc_init = linear_acc_init - accl_bias_;
  angular_vel_init = GetUnbiasedAngularVel(angular_vel_init, C_nb);

  // init process equation, in case of direct correct step:
  // 初始化过程方程，主要计算F和B
  UpdateProcessEquation(linear_acc_init, angular_vel_init);

  LOG(INFO) << std::endl
            << "Kalman Filter Inited at " << static_cast<int>(time_)
            << std::endl
            << "Init Position: " << pose_(0, 3) << ", " << pose_(1, 3) << ", "
            << pose_(2, 3) << std::endl
            << "Init Velocity: " << vel_.x() << ", " << vel_.y() << ", "
            << vel_.z() << std::endl;
}

bool ErrorStateKalmanFilter::Update(const IMUData &imu_data) {
  //
  // TODO!: understand ESKF update workflow
  //
  // update IMU buff:
  // 更新imu缓存
  if (time_ < imu_data.time) {
    // update IMU odometry:
    // 更新imu里程计
    Eigen::Vector3d linear_acc_mid;
    Eigen::Vector3d angular_vel_mid;
    imu_data_buff_.push_back(imu_data);

    // 更新里程计的估计值
    UpdateOdomEstimation(linear_acc_mid, angular_vel_mid);
    imu_data_buff_.pop_front();

    // update error estimation:
    // 更新误差的先验估计
    double T = imu_data.time - time_;
    UpdateErrorEstimation(T, linear_acc_mid, angular_vel_mid);

    // move forward:
    // 更新时间
    time_ = imu_data.time;

    return true;
  }

  return false;
}

bool ErrorStateKalmanFilter::Correct(const IMUData &imu_data,
                                     const MeasurementType &measurement_type,
                                     const Measurement &measurement) {
  static Measurement measurement_;

  // get time delta:
  // 获取时间
  double time_delta = measurement.time - time_;

  if (time_delta > -0.05) {
    // perform Kalman prediction:
    // 如果时间过大，则在进行一次更新
    if (time_ < measurement.time) {
      Update(imu_data);
    }

    // get observation in navigation frame:
    // 获得在导航坐标系下的观测
    measurement_ = measurement;
    measurement_.T_nb = init_pose_ * measurement_.T_nb;

    // correct error estimation:
    // 校正误差的估计值
    CorrectErrorEstimation(measurement_type, measurement_);

    // eliminate error:
    // 去除误差
    EliminateError();

    // reset error state:
    // 重设误差状态
    ResetState();

    return true;
  }

  LOG(INFO) << "ESKF Correct: Observation is not synced with filter. Skip, "
            << (int)measurement.time << " <-- " << (int)time_ << " @ "
            << time_delta << std::endl;

  return false;
}

/**
 * @brief  get odometry estimation
 * @param  pose, init pose
 * @param  vel, init vel
 * @return void
 */
void ErrorStateKalmanFilter::GetOdometry(Eigen::Matrix4f &pose,
                                         Eigen::Vector3f &vel) {
  // init:
  Eigen::Matrix4d pose_double = pose_;
  Eigen::Vector3d vel_double = vel_;

  // eliminate error:
  // a. position:
  pose_double.block<3, 1>(0, 3) =
      pose_double.block<3, 1>(0, 3) - X_.block<3, 1>(kIndexErrorPos, 0);
  // b. velocity:
  vel_double = vel_double - X_.block<3, 1>(kIndexErrorVel, 0);
  // c. orientation:
  Eigen::Matrix3d C_nn =
      Sophus::SO3d::exp(X_.block<3, 1>(kIndexErrorOri, 0)).matrix();
  pose_double.block<3, 3>(0, 0) = C_nn * pose_double.block<3, 3>(0, 0);

  // finally:
  pose_double = init_pose_.inverse() * pose_double;
  vel_double = init_pose_.block<3, 3>(0, 0).transpose() * vel_double;

  pose = pose_double.cast<float>();
  vel = vel_double.cast<float>();
}

/**
 * @brief  get covariance estimation
 * @param  cov, covariance output
 * @return void
 */
void ErrorStateKalmanFilter::GetCovariance(Cov &cov) {
  static int OFFSET_X = 0;
  static int OFFSET_Y = 1;
  static int OFFSET_Z = 2;

  // a. delta position:
  cov.pos.x = P_(kIndexErrorPos + OFFSET_X, kIndexErrorPos + OFFSET_X);
  cov.pos.y = P_(kIndexErrorPos + OFFSET_Y, kIndexErrorPos + OFFSET_Y);
  cov.pos.z = P_(kIndexErrorPos + OFFSET_Z, kIndexErrorPos + OFFSET_Z);

  // b. delta velocity:
  cov.vel.x = P_(kIndexErrorVel + OFFSET_X, kIndexErrorVel + OFFSET_X);
  cov.vel.y = P_(kIndexErrorVel + OFFSET_Y, kIndexErrorVel + OFFSET_Y);
  cov.vel.z = P_(kIndexErrorVel + OFFSET_Z, kIndexErrorVel + OFFSET_Z);

  // c. delta orientation:
  cov.ori.x = P_(kIndexErrorOri + OFFSET_X, kIndexErrorOri + OFFSET_X);
  cov.ori.y = P_(kIndexErrorOri + OFFSET_Y, kIndexErrorOri + OFFSET_Y);
  cov.ori.z = P_(kIndexErrorOri + OFFSET_Z, kIndexErrorOri + OFFSET_Z);

  // d. gyro. bias:
  cov.gyro_bias.x =
      P_(kIndexErrorGyro + OFFSET_X, kIndexErrorGyro + OFFSET_X);
  cov.gyro_bias.y =
      P_(kIndexErrorGyro + OFFSET_Y, kIndexErrorGyro + OFFSET_Y);
  cov.gyro_bias.z =
      P_(kIndexErrorGyro + OFFSET_Z, kIndexErrorGyro + OFFSET_Z);

  // e. accel bias:
  cov.accel_bias.x =
      P_(kIndexErrorAccel + OFFSET_X, kIndexErrorAccel + OFFSET_X);
  cov.accel_bias.y =
      P_(kIndexErrorAccel + OFFSET_Y, kIndexErrorAccel + OFFSET_Y);
  cov.accel_bias.z =
      P_(kIndexErrorAccel + OFFSET_Z, kIndexErrorAccel + OFFSET_Z);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::GetUnbiasedAngularVel(
    const Eigen::Vector3d &angular_vel, const Eigen::Matrix3d &R) {
  return angular_vel - gyro_bias_;
}

inline Eigen::Vector3d
ErrorStateKalmanFilter::GetUnbiasedLinearAcc(const Eigen::Vector3d &linear_acc,
                                             const Eigen::Matrix3d &R) {
  return R * (linear_acc - accl_bias_) - g_;
}

bool ErrorStateKalmanFilter::GetAngularDelta(const size_t index_curr,
                                             const size_t index_prev,
                                             Eigen::Vector3d &angular_delta,
                                             Eigen::Vector3d &angular_vel_mid) {
  // 确认索引的有效性
  if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
    return false;
  }

  // 获取当前帧和上一帧的imu数据
  const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
  const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

  // 获取时间差
  double delta_t = imu_data_curr.time - imu_data_prev.time;

  // 计算当前角速度
  Eigen::Vector3d angular_vel_curr = Eigen::Vector3d(
      imu_data_curr.angular_velocity.x, imu_data_curr.angular_velocity.y,
      imu_data_curr.angular_velocity.z);
  Eigen::Matrix3d R_curr = imu_data_curr.GetOrientationMatrix().cast<double>();
  // 无偏的当前速度
  angular_vel_curr = GetUnbiasedAngularVel(angular_vel_curr, R_curr);

  // 计算上一帧的角速度
  Eigen::Vector3d angular_vel_prev = Eigen::Vector3d(
      imu_data_prev.angular_velocity.x, imu_data_prev.angular_velocity.y,
      imu_data_prev.angular_velocity.z);
  Eigen::Matrix3d R_prev = imu_data_prev.GetOrientationMatrix().cast<double>();
  // 无偏的上一帧角速度
  angular_vel_prev = GetUnbiasedAngularVel(angular_vel_prev, R_prev);

  angular_delta = 0.5 * delta_t * (angular_vel_curr + angular_vel_prev);

  angular_vel_mid = 0.5 * (angular_vel_curr + angular_vel_prev);
  return true;
}

void ErrorStateKalmanFilter::UpdateOrientation(
    const Eigen::Vector3d &angular_delta, Eigen::Matrix3d &R_curr,
    Eigen::Matrix3d &R_prev) {
  // magnitude:
  // 幅值 旋转角
  double angular_delta_mag = angular_delta.norm();
  // direction:
  // 方向
  Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

  // build delta q:
  // 构建四元数
  double angular_delta_cos = cos(angular_delta_mag / 2.0);
  double angular_delta_sin = sin(angular_delta_mag / 2.0);
  Eigen::Quaterniond dq(angular_delta_cos,
                        angular_delta_sin * angular_delta_dir.x(),
                        angular_delta_sin * angular_delta_dir.y(),
                        angular_delta_sin * angular_delta_dir.z());
  Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));

  // update:
  // 更新
  q = q * dq;

  // write back:
  // 更新前一帧的姿态和当前帧的姿态:
  R_prev = pose_.block<3, 3>(0, 0);
  pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  R_curr = pose_.block<3, 3>(0, 0);
}

bool ErrorStateKalmanFilter::GetVelocityDelta(
    const size_t index_curr, const size_t index_prev,
    const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, double &T,
    Eigen::Vector3d &velocity_delta, Eigen::Vector3d &linear_acc_mid) {

  // 判断索引有效性
  if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
    return false;
  }

  // 获得当前帧和上一帧的imu数据
  const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
  const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

  // 时间间隔
  T = imu_data_curr.time - imu_data_prev.time;

  // TODO!:遗留错误修改
  // 获得当前帧和上一帧的加速度
  Eigen::Vector3d linear_acc_curr = Eigen::Vector3d(
      imu_data_curr.linear_acceleration.x, imu_data_curr.linear_acceleration.y,
      imu_data_curr.linear_acceleration.z);
  linear_acc_curr = GetUnbiasedLinearAcc(linear_acc_curr, R_curr);
  //linear_acc_curr = linear_acc_curr - accl_bias_;
  Eigen::Vector3d linear_acc_prev = Eigen::Vector3d(
      imu_data_prev.linear_acceleration.x, imu_data_prev.linear_acceleration.y,
      imu_data_prev.linear_acceleration.z);
  linear_acc_prev = GetUnbiasedLinearAcc(linear_acc_prev, R_prev);
  //linear_acc_prev = linear_acc_prev - accl_bias_;

  // mid-value acc can improve error state prediction accuracy:
  // TODO!:遗留错误修改
  //linear_acc_mid = 0.5 * (linear_acc_curr + linear_acc_prev);
  // 获得无偏加速度
  linear_acc_mid = (linear_acc_prev + linear_acc_curr) / 2 - accl_bias_;

  // 速度增量
  velocity_delta = T * linear_acc_mid;

  return true;
}

void ErrorStateKalmanFilter::UpdatePosition(
    const double &T, const Eigen::Vector3d &velocity_delta) {
  pose_.block<3, 1>(0, 3) += T * vel_ + 0.5 * T * velocity_delta;
  vel_ += velocity_delta;
}

void ErrorStateKalmanFilter::UpdateOdomEstimation(
    Eigen::Vector3d &linear_acc_mid, Eigen::Vector3d &angular_vel_mid) {
  //
  // TODO!: this is one possible solution to previous chapter, IMU Navigation,
  // 填充相应内容，并修改GetVelocityDelta内容
  // 对应3. 惯性解算内容
  // assignment
  //
  // get deltas:
  // 获取旋转向量
  Eigen::Vector3d angular_delta;
  GetAngularDelta(1, 0, angular_delta, angular_vel_mid);
  // update orientation:
  // 更新旋转姿态
  Eigen::Matrix3d R_curr, R_prev;
  UpdateOrientation(angular_delta, R_curr, R_prev);
  // get velocity delta:
  // 更新速度
  double T;
  Eigen::Vector3d velocity_delta;
  GetVelocityDelta(1, 0, R_curr, R_prev, T, velocity_delta, linear_acc_mid);
  // save mid-value unbiased linear acc for error-state update:

  // update position:
  // 更新位置
  UpdatePosition(T, velocity_delta);
}

void ErrorStateKalmanFilter::SetProcessEquation(const Eigen::Matrix3d &C_nb,
                                                const Eigen::Vector3d &f_n,
                                                const Eigen::Vector3d &w_b) {
  // TODO!: set process / system equation:
  // 填充相应内容，此内容与对应的Ft矩阵和Bt矩阵对应
  // a. set process equation for delta vel:
  F_.setZero();
  F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) = -C_nb * Sophus::SO3d::hat(f_n).matrix();
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorAccel) = -C_nb;
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) = -Sophus::SO3d::hat(w_b).matrix();
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) = -Eigen::Matrix3d::Identity();
  // b. set process equation for delta ori:
  B_.setZero();
  B_.block<3, 3>(kIndexErrorVel, kIndexNoiseAccel) = C_nb;
  B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = Eigen::Matrix3d::Identity();
}

void ErrorStateKalmanFilter::UpdateProcessEquation(
    const Eigen::Vector3d &linear_acc_mid,
    const Eigen::Vector3d &angular_vel_mid) {
  // TODO!:遗留错误修改
  // set linearization point:
  Eigen::Matrix3d C_nb = pose_.block<3, 3>(0, 0); // 用于转换body坐标系
  //Eigen::Vector3d f_n = linear_acc_mid + g_;
  //Eigen::Vector3d w_b = angular_vel_mid;

  // set process equation:
  // 设置状态方程
  //SetProcessEquation(C_nb, f_n, w_b);
  SetProcessEquation(C_nb, linear_acc_mid, angular_vel_mid);
}

void ErrorStateKalmanFilter::UpdateErrorEstimation(
    const double &T, const Eigen::Vector3d &linear_acc_mid,
    const Eigen::Vector3d &angular_vel_mid) {
  static MatrixF F_1st;
  static MatrixF F_2nd;
  // TODO!: update process equation:
  // 包括以下均填充了相应内容
  // 对应4. Kalman预测更新
  // 更新状态方程，设置F和B
  UpdateProcessEquation(linear_acc_mid, angular_vel_mid);
  // TODO!: get discretized process equations:
  // 
  F_1st = T * F_;
  MatrixF F = MatrixF::Identity() + F_1st;
  MatrixB B = MatrixB::Zero();
  B.block<3, 3>(kIndexErrorVel, kIndexNoiseAccel) = T * B_.block<3, 3>(kIndexErrorVel, kIndexNoiseAccel);
  B.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = T * B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro);
  B.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = std::sqrt(T) * B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel);
  B.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = std::sqrt(T) * B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro);
  // TODO!: perform Kalman prediction
  // 卡尔曼更新项
  P_ = F * P_ * F.transpose() + B * Q_ * B.transpose();
}

void ErrorStateKalmanFilter::CorrectErrorEstimationPose(
    const Eigen::Matrix4d &T_nb, Eigen::VectorXd &Y, Eigen::MatrixXd &G,
    Eigen::MatrixXd &K) {
  //
  // TODO!: set measurement:
  // 对应6. 有观测时的测量更新
  // 设置观测值
  // 填充如下内容
  Eigen::Vector3d dp = pose_.block<3, 1>(0, 3) - T_nb.block<3, 1>(0, 3);
  Eigen::Matrix3d dR = T_nb.block<3, 3>(0, 0).transpose() * pose_.block<3, 3>(0, 0);
  Eigen::Vector3d dtheta = Sophus::SO3d::vee(dR - Eigen::Matrix3d::Identity());
  // 测量值
  YPose_.block<3, 1>(0, 0) = dp;      // 位置
  YPose_.block<3, 1>(3, 0) = dtheta;  // 失准角
  Y = YPose_;                         // 构建总的测量
  // TODO!: set measurement equation:
  // 测量矩阵
  GPose_.setZero();
  GPose_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
  GPose_.block<3, 3>(3, kIndexErrorOri) = Eigen::Matrix3d::Identity();
  G = GPose_;
  CPose_.setZero();
  CPose_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  // TODO!: set Kalman gain:
  K = P_ * G.transpose() * (G * P_ * G.transpose() + CPose_ * RPose_ * CPose_.transpose()).inverse();
}

void ErrorStateKalmanFilter::CorrectErrorEstimation(
    const MeasurementType &measurement_type, const Measurement &measurement) {
  //
  // TODO!: understand ESKF correct workflow
  // 理解如下内容
  Eigen::VectorXd Y;
  Eigen::MatrixXd G, K;
  switch (measurement_type) {
  case MeasurementType::POSE:
    CorrectErrorEstimationPose(measurement.T_nb, Y, G, K);
    break;
  default:
    break;
  }

  // TODO!: perform Kalman correct:
  P_ = (MatrixP::Identity() - K * G) * P_;
  X_ = X_ + K * (Y - G * X_);
}

void ErrorStateKalmanFilter::EliminateError(void) {
  //
  // TODO!: correct state estimation using the state of ESKF
  // 填充下面内容
  // a. position: 位置
  // do it!
  pose_.block<3, 1>(0, 3) -= X_.block<3, 1>(kIndexErrorPos, 0);
  // b. velocity: 速度
  // do it!
  vel_ -= X_.block<3, 1>(kIndexErrorVel, 0);
  // c. orientation: 姿态
  // do it!
  Eigen::Matrix3d dtheta_cross = Sophus::SO3d::hat(X_.block<3, 1>(kIndexErrorOri, 0));
  pose_.block<3, 3>(0, 0) = pose_.block<3, 3>(0, 0) * (Eigen::Matrix3d::Identity() - dtheta_cross);
  Eigen::Quaterniond q_tmp(pose_.block<3, 3>(0, 0));
  q_tmp.normalize();
  pose_.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();
  // d. gyro bias: 零偏
  //if (IsCovStable(kIndexErrorGyro)) {
    //gyro_bias_ += X_.block<3, 1>(kIndexErrorGyro, 0);
    gyro_bias_ -= X_.block<3, 1>(kIndexErrorGyro, 0);
  //}

  // TODO !:遗留错误修改
  // e. accel bias: 零偏
  //if (IsCovStable(kIndexErrorAccel)) {
    //accl_bias_ += X_.block<3, 1>(kIndexErrorAccel, 0);
    accl_bias_ -= X_.block<3, 1>(kIndexErrorAccel, 0);
  //}
}

/**
 * @brief  is covariance stable
 * @param  INDEX_OFSET, state index offset
 * @param  THRESH, covariance threshold, defaults to 1.0e-5
 * @return void
 */
bool ErrorStateKalmanFilter::IsCovStable(const int INDEX_OFSET,
                                         const double THRESH) {
  for (int i = 0; i < 3; ++i) {
    if (P_(INDEX_OFSET + i, INDEX_OFSET + i) > THRESH) {
      return false;
    }
  }

  return true;
}

void ErrorStateKalmanFilter::ResetState(void) {
  // reset current state:
  X_ = VectorX::Zero();
}

void ErrorStateKalmanFilter::ResetCovariance(void) {
  P_ = MatrixP::Zero();

  P_.block<3, 3>(kIndexErrorPos, kIndexErrorPos) =
      COV.PRIOR.POSI * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorVel, kIndexErrorVel) =
      COV.PRIOR.VEL * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) =
      COV.PRIOR.ORI * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorGyro, kIndexErrorGyro) =
      COV.PRIOR.EPSILON * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorAccel, kIndexErrorAccel) =
      COV.PRIOR.DELTA * Eigen::Matrix3d::Identity();
}

/**
 * @brief  get Q analysis for pose measurement
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::GetQPose(Eigen::MatrixXd &Q, Eigen::VectorXd &Y) {
  // build observability matrix for position measurement:
  Y = Eigen::VectorXd::Zero(kDimState * kDimMeasurementPose);
  Y.block<kDimMeasurementPose, 1>(0, 0) = YPose_;
  for (int i = 1; i < kDimState; ++i) {
    QPose_.block<kDimMeasurementPose, kDimState>(i * kDimMeasurementPose, 0) =
        (QPose_.block<kDimMeasurementPose, kDimState>(
             (i - 1) * kDimMeasurementPose, 0) *
         F_);

    Y.block<kDimMeasurementPose, 1>(i * kDimMeasurementPose, 0) = YPose_;
  }

  Q = QPose_;
}

/**
 * @brief  update observability analysis
 * @param  measurement_type, measurement type
 * @return void
 */
void ErrorStateKalmanFilter::UpdateObservabilityAnalysis(
    const double &time, const MeasurementType &measurement_type) {
  // get Q:
  Eigen::MatrixXd Q;
  Eigen::VectorXd Y;
  switch (measurement_type) {
  case MeasurementType::POSE:
    GetQPose(Q, Y);
    break;
  default:
    break;
  }

  observability.time_.push_back(time);
  observability.Q_.push_back(Q);
  observability.Y_.push_back(Y);
}

/**
 * @brief  save observability analysis to persistent storage
 * @param  measurement_type, measurement type
 * @return void
 */
bool ErrorStateKalmanFilter::SaveObservabilityAnalysis(
    const MeasurementType &measurement_type) {
  // get fusion strategy:
  std::string type;
  switch (measurement_type) {
  case MeasurementType::POSE:
    type = std::string("pose");
    break;
  case MeasurementType::POSE_VEL:
    type = std::string("pose_velocity");
    break;
  case MeasurementType::POSI:
    type = std::string("position");
    break;
  case MeasurementType::POSI_VEL:
    type = std::string("position_velocity");
    break;
  default:
    return false;
    break;
  }

  // build Q_so:
  const int N = observability.Q_.at(0).rows();

  std::vector<std::vector<double>> q_data, q_so_data;

  Eigen::MatrixXd Qso(observability.Q_.size() * N, kDimState);
  Eigen::VectorXd Yso(observability.Y_.size() * N);

  for (size_t i = 0; i < observability.Q_.size(); ++i) {
    const double &time = observability.time_.at(i);

    const Eigen::MatrixXd &Q = observability.Q_.at(i);
    const Eigen::VectorXd &Y = observability.Y_.at(i);

    Qso.block(i * N, 0, N, kDimState) = Q;
    Yso.block(i * N, 0, N, 1) = Y;

    KalmanFilter::AnalyzeQ(kDimState, time, Q, Y, q_data);

    if (0 < i && (0 == i % 10)) {
      KalmanFilter::AnalyzeQ(kDimState, observability.time_.at(i - 5),
                             Qso.block((i - 10), 0, 10 * N, kDimState),
                             Yso.block((i - 10), 0, 10 * N, 1), q_so_data);
    }
  }

  std::string q_data_csv =
      WORK_SPACE_PATH + "/slam_data/observability/" + type + ".csv";
  std::string q_so_data_csv =
      WORK_SPACE_PATH + "/slam_data/observability/" + type + "_som.csv";

  KalmanFilter::WriteAsCSV(kDimState, q_data, q_data_csv);
  KalmanFilter::WriteAsCSV(kDimState, q_so_data, q_so_data_csv);

  return true;
}

} // namespace lidar_localization