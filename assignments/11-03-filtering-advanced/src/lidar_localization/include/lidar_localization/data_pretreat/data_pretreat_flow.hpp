/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */
#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <memory>
#include <ros/ros.h>
// subscriber
#include "lidar_localization/sensor_data/pose_data.hpp"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
// publisher
// a. synced lidar measurement
#include "lidar_localization/publisher/cloud_publisher.hpp"
// b. synced IMU measurement
#include "lidar_localization/publisher/imu_publisher.hpp"
// c. synced GNSS-odo measurement:
#include "lidar_localization/publisher/pos_vel_publisher.hpp"
// d. synced reference trajectory:
#include "lidar_localization/publisher/odometry_publisher.hpp"

// models
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"

namespace lidar_localization {
class DataPretreatFlow {
  public:
    /**
     * @brief 构造函数
     *        初始化velodyne激光雷达数据、OXTS IMU数据、OXTS 速度数据、OXTS GNSS数据、雷达到imu位姿变换的接收者
     *        初始化去畸变点云、imu数据、lidar位置速度数据、gnss数据的发布者
     *        初始化雷达点云畸变补偿功能模块
     * @param nh ros句柄
     * @param cloud_topic 去畸变点云将要发布的话题名称
     */  
    DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic);

    /**
     * @brief 该模块的主运行程序，依次运行如下功能
     *        1.读取接受者到的点云、imu、速度和gnss数据，并进行时间同步
     *        2.读取lidar传感器和gnss传感器的外参
     *        3.GNSS的初始化，取出GNSS数据队列的第一个数据，进行重设原点操作
     *        4.检测是否有点云、imu、速度和gnss数据后，确保数据存在，依次运行
     *          a.从数据之间的时间差检查数据的有效性，当全部有效再向下执行
     *          b.将所有数据转换到lidar坐标系下
     *          c.发布校正后的点云，同步后的imu、gnss和位置速度数据
     * @return 
     */    
    bool Run();

  private:
    /**
     * @brief 读取接受者到的点云、imu、速度和gnss数据，并进行时间同步
     * @return 存在以下几种情况返回结果是false
     *         1.未读取到雷达数据
     *         2.雷达未初始化(第一次时间同步成功便初始化成功)
     */    
    bool ReadData();

    /**
     * @brief 读取lidar传感器和gnss传感器的外参
     *        便于之后计算imu数据在lidar传感器坐标系位姿
     *        此操作仅进行一次
     * @return 外参读取失败返回false
     */    
    bool InitCalibration();

    /**
     * @brief GNSS的初始化，取出GNSS数据队列的第一个数据，进行重设原点操作
     *        此操作仅进行一次
     * @return 按照逻辑来说，永远为true
     */    
    bool InitGNSS();

    /**
     * @brief 检测是否有点云、imu、速度和gnss数据
     * @return 均有数据则返回ture
     */    
    bool HasData();

    /**
     * @brief 从imu、gnss、lidar和速度数据缓存中读取一帧数据设置为当前帧数据
     *        并从之间的时间差检查数据的有效性
     *        数据的时间检查是在lidar频率为10hz的假设上进行的
     *        当发现数据时间不一致，则丢弃这个不一致的数据
     *        当发现数据时间一致，则保存当前帧数据，并在缓冲中将当前帧数据丢弃
     * @return 当四个数据的时间差均在有效范围内，返回true，有一个不满足，返回flase
     */    
    bool ValidData();

    /**
     * @brief 获取GNSS数据的雷达坐标系下的位姿
     *        获取lidar此时的速度和加速度
     *        对lidar进行点云畸变校正
     * @return 均为true，可视作完成该功能的标识
     */    
    bool TransformData();

    /**
     * @brief 发布校正后的点云，同步后的imu、gnss和位置速度数据
     * @return 均为true，可视作完成该功能的标识
     */    
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;            // imu数据的接受者
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;  // 速度数据的接受者
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;        // 点云数据的接受者
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;          // GNSS数据的接受者
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;          // 雷达与imu传感器外参的接受者
    std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;
    
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;         // 去畸变点云的发布者
    std::shared_ptr<IMUPublisher> imu_pub_ptr_;             // 同步后imu数据的发布者
    std::shared_ptr<PosVelPublisher> pos_vel_pub_ptr_;      // 同步后位置速度数据的发布者
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;       // 同步后，Gnss数据的发布者
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_; // 点云去畸变模块

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();  // 雷达与imu传感器的外参

    std::deque<CloudData> cloud_data_buff_;         // 点云数据队列
    std::deque<IMUData> imu_data_buff_;             // 同步后imu数据队列
    std::deque<VelocityData> velocity_data_buff_;   // 同步后速度数据队列
    std::deque<GNSSData> gnss_data_buff_;           // 同步后gnss数据队列
    std::deque<PoseData> odom_data_buff_;

    CloudData current_cloud_data_;        // 当前点云数据
    IMUData current_imu_data_;            // 当前imu数据
    VelocityData current_velocity_data_;  // 当前速度数据
    GNSSData current_gnss_data_;          // 当前GNSS数据
    PoseData current_odom_data_;

    PosVelData pos_vel_;                  // 位置和速度数据
    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity(); // gnss在lidar坐标系下的位姿矩阵
};
}

#endif