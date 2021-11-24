/*
 * @Description: Error-State Kalman Filter for IMU-Lidar-GNSS-Odo fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_HPP_

#include "lidar_localization/models/kalman_filter/kalman_filter.hpp"

namespace lidar_localization {
/**
 * 误差卡尔曼滤波器
 */ 
class ErrorStateKalmanFilter : public KalmanFilter {
public:
  /**
   * @brief 1. parse config: 解析配置文件
   *          a. earth constants: 地球常量
   *          b. prior state covariance: 先验状态的协方差
   *          c. process noise: 过程噪声的协方差
   *          d. measurement noise: 测量噪声的协方差
   *        2. prompt: 输出读取的配置信息
   *        3. init filter: 初始化滤波器
   *          a. earth constants: 地球常量
   *          b. prior state & covariance: 重置状态和先验协方差矩阵:
   *          c. process noise: 过程噪声:
   *          d. measurement noise: 测量噪声:
   *          e. process equation: 过程方程:
   *          f. measurement equation: 测量方程:
   *        4. init soms:
   * @param Node &node 配置文件
   */
  ErrorStateKalmanFilter(const YAML::Node &node);

  /**
   * @brief  init filter
   *         初始化滤波器
   * @param  imu_data, input IMU measurements
   * @return true if success false otherwise
   */
  void Init(const Eigen::Vector3d &vel, const IMUData &imu_data);

  /**
   * @brief  Kalman update 卡尔曼滤波器更新
   * @param  imu_data, input IMU measurements         
   * @return true if success false otherwise
   */
  bool Update(const IMUData &imu_data);

  /**
   * @brief  Kalman correction, pose measurement
   *         卡尔曼量测更新
   * @param  imu_data imu数据
   * @param  measurement_type, input measurement type
   * @param  measurement, input measurement
   * @return void
   */
  bool Correct(const IMUData &imu_data, const MeasurementType &measurement_type,
               const Measurement &measurement);

  /**
   * @brief  Kalman correction, pose measurement and other measurement in body
   * frame
   * @param  T_nb, pose measurement
   * @param  v_b, velocity or magnetometer measurement
   * @return void
   */
  bool Correct(const IMUData &imu_data, const double &time,
               const MeasurementType &measurement_type,
               const Eigen::Matrix4f &T_nb, const Eigen::Vector3f &v_b);

  /**
   * @brief  get odometry estimation
   * @param  pose, init pose
   * @param  vel, init vel
   * @return void
   */
  void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel);

  /**
   * @brief  get covariance estimation
   * @param  cov, covariance output
   * @return void
   */
  void GetCovariance(Cov &cov);

  /**
   * @brief  update observability analysis
   * @param  time, measurement time
   * @param  measurement_type, measurement type
   * @return void
   */
  void UpdateObservabilityAnalysis(const double &time,
                                   const MeasurementType &measurement_type);

  /**
   * @brief  save observability analysis to persistent storage
   * @param  measurement_type, measurement type
   * @return void
   */
  bool SaveObservabilityAnalysis(const MeasurementType &measurement_type);

private:
  // indices:
  static constexpr int kDimState{15};             // 状态量的数量 3*15

  static constexpr int kIndexErrorPos{0};         // 先验状态量delta p的方差矩阵位置
  static constexpr int kIndexErrorVel{3};         // 先验状态量delta v的方差矩阵位置
  static constexpr int kIndexErrorOri{6};         // 先验状态量delta theta的方差矩阵位置
  static constexpr int kIndexErrorAccel{9};       // 先验状态量delta ba的方差矩阵位置
  static constexpr int kIndexErrorGyro{12};       // 先验状态量delta bw的方差矩阵位置

  static constexpr int kDimProcessNoise{12};      // 输入噪声方差的维度

  static constexpr int kIndexNoiseAccel{0};       // 过程噪声a的噪声矩阵位置
  static constexpr int kIndexNoiseGyro{3};        // 过程噪声w的噪声矩阵位置
  static constexpr int kIndexNoiseBiasAccel{6};   // 过程噪声ba的噪声矩阵位置
  static constexpr int kIndexNoiseBiasGyro{9};    // 过程噪声bw的噪声矩阵位置

  // dimensions:
  static constexpr int kDimMeasurementPose{6};    // 输入噪声方差矩阵位置
  static constexpr int kDimMeasurementPoseNoise{6};

  static constexpr int kDimMeasurementPoseVel{9};       // 运动约束
  static constexpr int kDimMeasurementPoseVelNoise{9};

  static constexpr int kDimMeasurementPosiVel{/*6*/9};       // 编码器
  static constexpr int kDimMeasurementPosiVelNoise{/*6*/9};

  // state:
  using VectorX=Eigen::Matrix<double, kDimState, 1>;            // 状态量X
  using MatrixP=Eigen::Matrix<double, kDimState, kDimState>;    // 状态量的方差P

  // process equation:
  using MatrixF=Eigen::Matrix<double, kDimState, kDimState>;                // 状态方程的X矩阵
  using MatrixB=Eigen::Matrix<double, kDimState, kDimProcessNoise>;         // 状态方程的B矩阵
  using MatrixQ=Eigen::Matrix<double, kDimProcessNoise, kDimProcessNoise>;  // 输入噪声的方差矩阵

  // measurement equation:
  using MatrixGPose=Eigen::Matrix<double, kDimMeasurementPose,kDimState> ;                    // 观测方程的G矩阵
  using MatrixCPose=Eigen::Matrix<double, kDimMeasurementPose,kDimMeasurementPoseNoise>;      // 观测方程的C矩阵
  using MatrixRPose=Eigen::Matrix<double, kDimMeasurementPoseNoise,kDimMeasurementPoseNoise>; // 测量噪声方差的维度

  using MatrixGPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVel,kDimState> ;                      // 融合运动约束
  using MatrixCPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVel,kDimMeasurementPoseVelNoise>;
  using MatrixRPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVelNoise,kDimMeasurementPoseVelNoise>;

  using MatrixGPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVel,kDimState> ;                      // 融合编码器
  using MatrixCPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVel,kDimMeasurementPosiVelNoise>;
  using MatrixRPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVelNoise,kDimMeasurementPosiVelNoise>;



  // measurement:
  using VectorYPose=Eigen::Matrix<double, kDimMeasurementPose, 1>;
  using VectorYPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVel, 1>;
  using VectorYPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVel, 1>;

  // Kalman gain:
  using MatrixKPose=Eigen::Matrix<double, kDimState, kDimMeasurementPose>;
  using MatrixKPoseVel=Eigen::Matrix<double, kDimState, kDimMeasurementPoseVel>;
  using MatrixKPosiVel=Eigen::Matrix<double, kDimState, kDimMeasurementPosiVel>;

  // state observality matrix:
  using MatrixQPose=Eigen::Matrix<double, kDimState * kDimMeasurementPose, kDimState>;
  using MatrixQPoseVel=Eigen::Matrix<double, kDimState * kDimMeasurementPoseVel, kDimState>;
  using MatrixQPosiVel=Eigen::Matrix<double, kDimState * kDimMeasurementPosiVel, kDimState>;

  /**
   * @brief  get unbiased angular velocity in body frame
   *         获得无偏的角速度在body坐标系下
   * @param  angular_vel, angular velocity measurement
   * @param  R, corresponding orientation of measurement
   * @return unbiased angular velocity in body frame
   */
  Eigen::Vector3d GetUnbiasedAngularVel(const Eigen::Vector3d &angular_vel,
                                        const Eigen::Matrix3d &R);
  /**
   * @brief  get unbiased linear acceleration in navigation frame
   *         获得无偏的线速度在navigation坐标系下
   * @param  linear_acc, linear acceleration measurement
   * @param  R, corresponding orientation of measurement
   * @return unbiased linear acceleration in navigation frame
   */
  Eigen::Vector3d GetUnbiasedLinearAcc(const Eigen::Vector3d &linear_acc,
                                       const Eigen::Matrix3d &R);

  /**
   * @brief  get angular delta
   *         获得角度增量
   * @param  index_curr, current imu measurement buffer index 
   * @param  index_prev, previous imu measurement buffer index
   * @param  angular_delta, angular delta output
   * @param  angular_vel_mid  output mid-value unbiased angular vel
   * @return true if success false otherwise
   */
  bool GetAngularDelta(const size_t index_curr, const size_t index_prev,
                       Eigen::Vector3d &angular_delta,
                       Eigen::Vector3d &angular_vel_mid);
  /**
   * @brief  get velocity delta
   *         更新速度
   * @param  index_curr, current imu measurement buffer index
   * @param  index_prev, previous imu measurement buffer index
   * @param  R_curr, corresponding orientation of current imu measurement
   * @param  R_prev, corresponding orientation of previous imu measurement
   * @param  velocity_delta, velocity delta output
   * @param  linear_acc_mid, mid-value unbiased linear acc
   * @return true if success false otherwise
   */
  bool GetVelocityDelta(const size_t index_curr, const size_t index_prev,
                        const Eigen::Matrix3d &R_curr,
                        const Eigen::Matrix3d &R_prev, double &T,
                        Eigen::Vector3d &velocity_delta,
                        Eigen::Vector3d &linear_acc_mid);
  /**
   * @brief  update orientation with effective rotation angular_delta
   *         更新旋转姿态，基于四元素
   * @param  angular_delta, effective rotation  旋转矢量
   * @param  R_curr, current orientation
   * @param  R_prev, previous orientation
   * @return void
   */
  void UpdateOrientation(const Eigen::Vector3d &angular_delta,
                         Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev);
  /**
   * @brief  update orientation with effective velocity change velocity_delta
   *         更新位置
   * @param  T 时间间隔
   * @param  velocity_delta, effective velocity change
   * @return void
   */
  void UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta);
  /**
   * @brief  update IMU odometry estimation
   *         更新里程计的估计值
   * @param  linear_acc_mid, output mid-value unbiased linear acc
   * @param  angular_vel_mid, output mid-value unbiased angular vel
   * @return void
   */
  void UpdateOdomEstimation(Eigen::Vector3d &linear_acc_mid,
                            Eigen::Vector3d &angular_vel_mid);

  /**
   * @brief  set process equation
   *         设置状态方程,设置F和B
   *         在此不需要转换到navigation坐标系
   * @param  C_nb, rotation matrix, body frame -> navigation frame
   * @param  f_n, accel measurement in body(navigation) frame
   * @param  w_n  angular measurement in body(navigation) frame
   * @return void
   */
  void SetProcessEquation(const Eigen::Matrix3d &C_nb,
                          const Eigen::Vector3d &f_n,
                          const Eigen::Vector3d &w_n);
  /**
   * @brief  update process equation
   *         更新状态方程
   * @param  linear_acc_mid, input mid-value unbiased linear acc
   * @param  angular_vel_mid input mid-value unbiased angular vel
   * @return void
   */
  void UpdateProcessEquation(const Eigen::Vector3d &linear_acc_mid,
                             const Eigen::Vector3d &angular_vel_mid);

  /**
   * @brief  update error estimation
   *         Kalman预测更新
   * @param  linear_acc_mid, input mid-value unbiased linear acc
   * @param  angular_vel_mid input mid-value unbiased angular vel
   * @return void
   */
  void UpdateErrorEstimation(const double &T,
                             const Eigen::Vector3d &linear_acc_mid,
                             const Eigen::Vector3d &angular_vel_mid);

  /**
   * @brief  correct error estimation using pose measurement
   *         基于观测来改进误差估计
   * @param  T_nb, input pose measurement
   * @param  Y  观测值，一般有位置和失准角组成
   * @param  G  测量矩阵
   * @param  K  卡尔曼增益
   * @return void
   */
  void CorrectErrorEstimationPose(const Eigen::Matrix4d &T_nb,
                                  Eigen::VectorXd &Y, Eigen::MatrixXd &G,
                                  Eigen::MatrixXd &K);

  /**
   * @brief  correct error estimation using pose and body velocity measurement
   * @param  T_nb, input pose measurement
   * @param  v_b, input velocity measurement
   * @return void
   */
  void CorrectErrorEstimationPoseVel(
      const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b, const Eigen::Vector3d &w_b,
      Eigen::VectorXd &Y, Eigen::MatrixXd &G, Eigen::MatrixXd &K
  );

  /**
    * @brief  correct error estimation using navigation position and body velocity measurement
    * @param  T_nb, input position measurement
    * @param  v_b, input velocity measurement
    * @return void
    */
  void CorrectErrorEstimationPosiVel(
      const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b, const Eigen::Vector3d &w_b,
      Eigen::VectorXd &Y, Eigen::MatrixXd &G, Eigen::MatrixXd &K
  );

  /**
   * @brief  correct error estimation
   *         校正误差的估计值
   * @param  measurement_type, measurement type
   * @param  measurement, input measurement
   * @return void
   */
  void CorrectErrorEstimation(const MeasurementType &measurement_type,
                              const Measurement &measurement);

  /**
   * @brief  eliminate error
   *         修正名义值: 平移，速度，旋转和零偏
   *         修正方式为预测值减去误差值
   * @param  void
   * @return void
   */
  void EliminateError(void);

  /**
   * @brief  is covariance stable
   * @param  INDEX_OFSET, state index offset
   * @param  THRESH, covariance threshold, defaults to 1.0e-5
   * @return void
   */
  bool IsCovStable(const int INDEX_OFSET, const double THRESH = 1.0e-5);

  /**
   * @brief  reset filter state
   *         重设滤波器状态常量，即将X重设为0向量
   * @param  void
   * @return void
   */
  void ResetState(void);
  /**
   * @brief  reset filter covariance
   * @param  void
   * @return void
   */
  void ResetCovariance(void);

  /**
   * @brief  get Q analysis for pose measurement
   * @param  void
   * @return void
   */
  void GetQPose(Eigen::MatrixXd &Q, Eigen::VectorXd &Y);

  // odometry estimation from IMU integration:
  // IMU惯导的里程计估计
  Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity(); // 初始位姿，用作定位坐标系的原点

  Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();      // 位姿
  Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();           // 速度
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accl_bias_ = Eigen::Vector3d::Zero();

  // state: 状态量
  VectorX X_ = VectorX::Zero();       // 状态量，delta[p,v,theta,ba,bw]
  MatrixP P_ = MatrixP::Zero();       // 方差矩阵，对应状态量的方差
  // process & measurement equations:
  MatrixF F_ = MatrixF::Zero();       // 状态方程的F矩阵
  MatrixB B_ = MatrixB::Zero();       // 状态方程的B矩阵
  MatrixQ Q_ = MatrixQ::Zero();       // 过程噪声矩阵

  MatrixGPose GPose_ = MatrixGPose::Zero();   // 观测方程的G矩阵
  MatrixCPose CPose_ = MatrixCPose::Zero();   // 观测方程的C矩阵
  MatrixRPose RPose_ = MatrixRPose::Zero();   // 测量噪声矩阵
  MatrixQPose QPose_ = MatrixQPose::Zero();   // 输入噪声的方差矩阵


  MatrixGPoseVel GPoseVel_ = MatrixGPoseVel::Zero();    // 融合运动约束
  MatrixCPoseVel CPoseVel_ = MatrixCPoseVel::Zero();
  MatrixRPoseVel RPoseVel_ = MatrixRPoseVel::Zero();
  MatrixQPoseVel QPoseVel_ = MatrixQPoseVel::Zero();
  Eigen::Matrix<double, kDimMeasurementPoseVel - 1, 1> YPoseVelCons_ =
      Eigen::Matrix<double, kDimMeasurementPoseVelNoise - 1, 1>::Zero();
  Eigen::Matrix<double, kDimMeasurementPoseVel - 1, kDimState> GPoseVelCons_ =
      Eigen::Matrix<double, kDimMeasurementPoseVel - 1, kDimState>::Zero();
  Eigen::Matrix<double, kDimMeasurementPoseVel - 1, kDimMeasurementPoseVelNoise> CPoseVelCons_ =
      Eigen::Matrix<double, kDimMeasurementPoseVel - 1, kDimMeasurementPoseVelNoise>::Zero();

  MatrixGPosiVel GPosiVel_ = MatrixGPosiVel::Zero();    // 融合编码器
  MatrixCPosiVel CPosiVel_ = MatrixCPosiVel::Zero();
  MatrixRPosiVel RPosiVel_ = MatrixRPosiVel::Zero();
  MatrixQPosiVel QPosiVel_ = MatrixQPosiVel::Zero();

  // measurement:
  VectorYPose YPose_ = VectorYPose::Zero();
  VectorYPoseVel YPoseVel_ = VectorYPoseVel::Zero(); 
  VectorYPosiVel YPosiVel_ = VectorYPosiVel::Zero(); 
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_HPP_