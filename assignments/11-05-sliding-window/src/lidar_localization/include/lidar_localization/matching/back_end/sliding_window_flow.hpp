/*
 * @Description: LIO localization backend workflow, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_FLOW_HPP_
#define LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_FLOW_HPP_

#include <ros/ros.h>

//
// subscribers:
//
// a. lidar odometry, map matching pose & GNSS reference position:
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
// b. IMU measurement, for pre-integration:
#include "lidar_localization/subscriber/imu_subscriber.hpp"

#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/key_frame_publisher.hpp"
#include "lidar_localization/publisher/key_frames_publisher.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"

#include "lidar_localization/matching/back_end/sliding_window.hpp"


namespace lidar_localization {

class SlidingWindowFlow {
public:
    /**
     * @brief 初始化发布者、接收者和滑窗功能
     * @param nh ros句柄
     * @return void
     */
    SlidingWindowFlow(ros::NodeHandle& nh);

    /**
     * @brief  主运行程序
     * @param  void
     * @return 无数据返回false
     */    
    bool Run();

    /**
     * @brief  保存优化后的里程计位姿
     * @param  void
     * @return 成功执行完成后返回true
     */    
    bool SaveOptimizedTrajectory();
    
  private:
    /**
     * @brief  读取数据
     * @param  void
     * @return 读取数据操作完成返回true
     */    
    bool ReadData();

    /**
     * @brief  确保有数据
     * @param  void
     * @return 有数据返回true
     */    
    bool HasData();

    /**
     * @brief  确保数据有效性
     *         通过时间戳来确保
     * @param  void
     * @return 数据均有效返回true
     */    
    bool ValidData();


    bool UpdateIMUPreIntegration(void);


    bool UpdateBackEnd();

    /**
     * @brief  发布数据
     * @param  void
     * @return 成功发布数据后返回true
     */    
    bool PublishData();

  private:
    //
    // subscribers:
    // 接受者及其接收的数据
    //
    // a. lidar odometry:
    // a. 雷达里程计
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;
    std::deque<PoseData> laser_odom_data_buff_;
    // b. map matching odometry:
    // b. 地图匹配
    std::shared_ptr<OdometrySubscriber> map_matching_odom_sub_ptr_;
    std::deque<PoseData> map_matching_odom_data_buff_;
    // c. IMU measurement, for pre-integration:
    // c. IMU同步数据和IMU原始数据  
    std::shared_ptr<IMUSubscriber> imu_raw_sub_ptr_;
    std::deque<IMUData> imu_raw_data_buff_;
    std::shared_ptr<IMUSubscriber> imu_synced_sub_ptr_;
    std::deque<IMUData> imu_synced_data_buff_;
    // d. GNSS position:
    // d. GNSS 位姿
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
    std::deque<PoseData> gnss_pose_data_buff_;

    //
    // publishers:
    // 发布者
    //
    std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;
    std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
    std::shared_ptr<KeyFramesPublisher> optimized_trajectory_pub_ptr_;
    std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_;

    //
    // backend:
    // 滑窗
    //
    std::shared_ptr<SlidingWindow> sliding_window_ptr_;

    //
    // synced data:
    // 同步的数据
    //
    PoseData current_laser_odom_data_;
    PoseData current_map_matching_odom_data_;
    IMUData current_imu_data_;
    PoseData current_gnss_pose_data_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_FLOW_HPP_