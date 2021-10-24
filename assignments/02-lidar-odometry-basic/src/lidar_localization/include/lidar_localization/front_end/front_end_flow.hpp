/*
 * @Description: frone-end workflow
 * @Author: Ge Yao
 * @Date: 2021-04-03
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_

#include <memory>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

//
// TF tree:
//
#include "lidar_localization/subscriber/tf_listener.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"

//
// subscribers:
//
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"

//
// publishers:
//
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

//
// algorithm core:
//
#include "lidar_localization/front_end/front_end.hpp"

namespace lidar_localization {

class FrontEndFlow {
  public:
    /**
     * @brief 构造函数
     *        读取配置文件，初始化数据的发布和接收
     *        初始化当前帧、局部地图和全局地图
     * @param nh ros句柄 
     */ 
    FrontEndFlow(ros::NodeHandle& nh);

    /**
     * @brief 里程计运行的全过程
     *        包含数据读取、里程计更新、位姿和地图的发布
     * @return 运行成功则返回true
     */ 
    bool Run();

    /**
     * @brief 保存地图，跳转到前端里程计中
     * @return 不管怎么样都返回true
     *         能作为完成该函数的标识
     */ 
    bool SaveMap();

    /**
     * @brief 发布全局地图
     * @return 不管怎么样都返回true
     *         能作为完成该函数的标识
     */ 
    bool PublishGlobalMap();

  private:
    /**
     * @brief 初始化发布者和接收者
     *        接收lidar到imu的位姿变换
     *        发布map到lidar的位姿变换
     *        接收点云数据
     *        接收imu数据
     *        接收GNSS数据
     * @param nh ros用句柄
     * @param config_node 配置文件参数
     * @return 不管怎么样都返回true
     *         能作为完成该函数的标识
     */ 
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);

    /**
     * @brief 读取数据
     *        在每一个循环中，均会对雷达、gnss和imu缓存的数据进行一次读入
     * @return 存在两种情况返回false
     *         1. 没有雷达数据
     *         2. 时间同步未成功
     */ 
    bool ReadData();

    /**
     * @brief 读取雷达和imu的外参
     *        成功读取一次之后就不会再读取
     * @return 如果成功读取到雷达和imu的外参，则返回ture
     */ 
    bool InitCalibration();

    /**
     * @brief 初始化GNSS，主要是对GNSS进行原点初始化
     *        初始化成功后就不会再进行初始化判别
     * @return 如果成功对GNSS原点进行初始化，则返回true
     */ 
    bool InitGNSS();

    /**
     * @brief 检查是否有点云、imu和gnss数据
     * @return 如果点云、imu和gnss数据均有，则返回true
     */ 
    bool HasData();

    /**
     * @brief 检查当前数据的有效性
     *        如果点云数据慢，则丢弃一帧点云，并进行下一次循环的判别
     *        如果imu数据慢，则丢弃一帧imu和gnss，并进行下一次循环的判别
     *        若数据有效，丢弃所有队列的第一个数据
     * @return 若imu和点云时间戳相近，则返回true
     */ 
    bool ValidData();

    /**
     * @brief 更新Gnss里程计
     *        以Gnss数据的东北天坐标和航向信息为相应位姿
     *        同时，将Gnss的坐标投影到雷达坐标系
     * @return 不管怎么样都返回true
     *         能作为完成该函数的标识
     */ 
    bool UpdateGNSSOdometry();

    /**
     * @brief 更新雷达里程计
     *        初始化时，以GNSS信号作为初始位姿
     *        进入前端里程计进行更新
     * @return 不管怎么样都返回true
     *         能作为完成该函数的标识
     */ 
    bool UpdateLaserOdometry();

    /**
     * @brief 发布数据，主要用于显示
     *        激光里程计数据
     *        Gnss里程计数据
     *        当前扫描帧
     *        当前帧的点云
     *        局部地图
     * @return 不管怎么样都返回true
     *         能作为完成该函数的标识
     */ 
    bool PublishData();

    /**
     * @brief 打开保存轨迹的文件
     *        保存激光里程计数据和gnss数据
     * @return 若打开文件失败则返回false
     */ 
    bool SaveTrajectory();

  private:
    std::shared_ptr<TFListener> lidar_to_imu_tf_sub_ptr_; // TF的接受者，内含imu的frame_id（imu_link）和lidar的frame_id（velo_link）
    std::shared_ptr<TFBroadCaster> lidar_to_map_tf_pub_ptr_;  // TF的发布者，内含lidar的frame_id（velo_link），发布到/map中

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;  // lidar的话题名字/kitti/velo/pointcloud 和队列长度 1000000
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;      // imu的话题名字/kitti/oxts/imu 和队列长度 1000000
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;    // GNSS的话题名字/kitti/oxts/gps/fix 和队列长度 1000000

    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;           // 发布当前点云帧到/map
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;       // 发布局部地图到/map
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;      // 发布全局地图到/map

    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;   // 发布laser_odom, map, lidar
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;         // 发布gnss、map、lidar

    std::shared_ptr<FrontEnd> front_end_ptr_;                 // 前端里程计

    std::deque<CloudData> cloud_data_buff_;                   // 点云数据队列
    std::deque<IMUData> imu_data_buff_;                       // imu数据队列
    std::deque<GNSSData> gnss_data_buff_;                     // gnss数据队列

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();  // 雷达到imu的位姿
    CloudData current_cloud_data_;  // 当前帧的点云数据
    IMUData current_imu_data_;      // 当前帧的imu数据
    GNSSData current_gnss_data_;    // 当前帧的gnss数据

    CloudData::CLOUD_PTR local_map_ptr_;    // 局部地图
    CloudData::CLOUD_PTR global_map_ptr_;   // 全局地图
    CloudData::CLOUD_PTR current_scan_ptr_; // 当前扫描数据

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();  // 雷达里程计当前位姿
    Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();   // gnss里程计当前位姿
};

}

#endif