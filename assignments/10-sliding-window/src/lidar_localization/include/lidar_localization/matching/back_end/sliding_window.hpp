/*
 * @Description: LIO localization backend, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_HPP_
#define LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_HPP_

#include <string>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "lidar_localization/sensor_data/pose_data.hpp"
#include "lidar_localization/sensor_data/imu_data.hpp"

#include "lidar_localization/sensor_data/key_frame.hpp"

#include "lidar_localization/models/pre_integrator/imu_pre_integrator.hpp"

#include "lidar_localization/models/sliding_window/ceres_sliding_window.hpp"


namespace lidar_localization {

class SlidingWindow {
  public:
    /**
     * @brief  通过配置文件进行初始化
     * @param  void
     * @return void
     */    
    SlidingWindow();

    /**
     * @brief  进行imu预积分的更新
     * @param  IMUData
     * @return 完成imu预积分的更新则返回true
     */    
    bool UpdateIMUPreIntegration(const IMUData &imu_data);

    /**
     * @brief  滑窗的更新，依次进行如下操作
     *         1. 重置标识位
     *         2. 判断是否有新关键帧，如果有，则执行
     *            a. 滑窗更新
     *            b. 位姿优化
     * @param  
     * @return 
     */    
    bool Update(
      const PoseData &laser_odom,
      const PoseData &map_matching_odom,
      const IMUData &imu_data, 
      const PoseData& gnss_pose
    );

    /**
     * @brief  是否有新的关键帧
     * @param  void
     * @return 有新的关键帧则返回true
     */    
    bool HasNewKeyFrame();

    /**
     * @brief  是否有新的优化位姿
     * @param  void
     * @return 有新的优化位姿则返回true
     */    
    bool HasNewOptimized();
    
    /**
     * @brief  获得最后一帧关键帧
     * @param  key_frame, 关键帧
     * @return void
     */    
    void GetLatestKeyFrame(KeyFrame& key_frame);

    /**
     * @brief  获得最后一帧关键帧的Gnss位置
     * @param  key_frame, 关键帧
     * @return void
     */    
    void GetLatestKeyGNSS(KeyFrame& key_frame);

    /**
     * @brief  获得最后一帧里程计的位姿
     * @param  key_frame 关键帧
     * @return void
     */    
    void GetLatestOptimizedOdometry(KeyFrame& key_frame);

    /**
     * @brief  获得优化后的关键帧队列
     * @param  key_frames_deque 关键帧队列
     * @return void
     */    
    void GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque);

    /**
     * @brief  保存位姿，依次保存雷达里程计、优化后位姿和真实位姿
     * @param  void
     * @return 完成保存则返回true
     */    
    bool SaveOptimizedTrajectory();

  private:
    /**
     * @brief  从配置文件读取相关参数
     * @param  void
     * @return 完成读取则返回true
     */    
    bool InitWithConfig();

    /**
     * @brief  读取文件保存路径
     * @param  config_node 配置文件
     * @return 完成读取则返回true
     */    
    bool InitDataPath(const YAML::Node& config_node);

    /**
     * @brief  读取关键帧相关参数
     * @param  config_node 配置文件
     * @return 完成读取则返回true
     */    
    bool InitKeyFrameSelection(const YAML::Node& config_node);

    /**
     * @brief  读取滑窗相关参数，以及相应噪声
     * @param  config_node 配置文件
     * @return 完成读取则返回true
     */    
    bool InitSlidingWindow(const YAML::Node& config_node);

    /**
     * @brief  读取预积分参数，即是否使用预积分
     * @param  config_node 配置文件
     * @return 完成读取则返回true
     */    
    bool InitIMUPreIntegrator(const YAML::Node& config_node);

    /**
     * @brief  重设是否有新的关键帧的标识和是否有新的雷达位姿的标识
     * @param  void
     * @return void
     */    
    void ResetParam();

    /**
     * @brief  根据传入位姿和路径，将位姿写入路径
     * @param  ofs, 文件位置
     * @param  pose, 位姿
     * @return 完成保存则返回true
     */    
    bool SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose);
    
    /**
     * @brief  判断是否有关键帧，以下情况需要添加关键帧
     *         1. 第一帧
     *         2. 距离达到阈值
     *         3. 时间达到阈值
     *         如果为关键帧，则进行如下操作
     *         1. imu预积分更新
     *         2. 当前关键帧信息更新
     *         3. 关键帧队列更新
     * @param  laser_odom 雷达里程计位姿
     * @param  map_matching_odom 地图匹配位姿
     * @param  imu_data imu数据
     * @param  gnss_pose gnss数据
     * @return 如果为关键帧，则为true
     */    
    bool MaybeNewKeyFrame(
      const PoseData& laser_odom, 
      const PoseData &map_matching_odom,
      const IMUData &imu_data,
      const PoseData& gnss_pose
    );
    
    bool Update(void);
    bool MaybeOptimized();

  private:
    std::string trajectory_path_ = ""; // 位姿保存路径

    bool has_new_key_frame_ = false;  // 是否有新的关键帧的标识
    bool has_new_optimized_ = false;  // 是否有新的雷达位姿的标识

    KeyFrame current_key_frame_;          // 当前关键帧
    PoseData current_map_matching_pose_;  // 当前地图匹配位姿
    KeyFrame current_key_gnss_;           // 当前关键帧的GNSS位姿

    // key frame buffer:
    // 新的关键帧内含缓存区
    struct {
      std::deque<KeyFrame> lidar;
      std::deque<KeyFrame> optimized;
      std::deque<KeyFrame> reference;
    } key_frames_;

    // pre-integrator:
    // 预积分相关
    std::shared_ptr<IMUPreIntegrator> imu_pre_integrator_ptr_;
    IMUPreIntegrator::IMUPreIntegration imu_pre_integration_;

    // key frame config:
    // 关键帧相关的参数
    struct {
      float max_distance;
      float max_interval;
    } key_frame_config_;

    // optimizer:
    // ceres 进行优化功能
    std::shared_ptr<CeresSlidingWindow> sliding_window_ptr_;

    // measurement config:
    // 测量配置
    struct MeasurementConfig {
        struct {
          bool map_matching = false;
          bool imu_pre_integration = false;
        } source; // 位姿优化量，地图匹配的位姿和imu预积分的位姿

        struct {
          Eigen::VectorXd lidar_odometry;
          Eigen::VectorXd map_matching;
          Eigen::VectorXd gnss_position;
        } noise; // 噪声
    };
    MeasurementConfig measurement_config_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_HPP_