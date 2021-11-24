/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/sensor_data/pose_data.hpp"
#include <deque>
#include <memory>

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    // subscribers:
    // 接受者
    // a. velodyne measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/velodyne_points", 100000);
    // b. OXTS IMU:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/imu/data", 1000000);
    // c. OXTS velocity:
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    // d. OXTS GNSS:
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/navsat/fix", 1000000);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");
    odom_sub_ptr_=std::make_shared<OdometrySubscriber>(nh, "/navsat/odom", 1000000);

    // publishers:
    // 发布者
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "/velo_link", 100);
    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
    pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(nh, "/synced_pos_vel", "/map", "/imu_link", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/velo_link", 100);

    // motion compensation for lidar measurement:
    // 雷达的点云畸变补偿
    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
    if (!ReadData())
      return false;

    if (!InitCalibration())
      return false;

    if (!InitGNSS())
      return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        TransformData();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    static std::deque<IMUData> unsynced_imu_;           // 未同步的imu数据
    static std::deque<VelocityData> unsynced_velocity_; // 未同步的速度数据
    static std::deque<GNSSData> unsynced_gnss_; // 未同步的gnss数据
    static std::deque<PoseData> unsynced_odom_;

    // fetch lidar measurements from buffer:
    // 从缓存中取得点云、imu、速度和gnss数据
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(unsynced_imu_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);
    odom_sub_ptr_->ParseData(unsynced_odom_);

    // 仅在雷达数据不为0的时刻进行数据同步操作
    if (cloud_data_buff_.size() == 0)
        return false;

    // use timestamp of lidar measurement as reference:
    // 使用激光雷达的时间戳作为时间同步的基准
    double cloud_time = cloud_data_buff_.front().time;
    // sync IMU, velocity and GNSS with lidar measurement:
    // find the two closest measurement around lidar measurement time
    // then use linear interpolation to generate synced measurement:
    // 对IMU、速度数据、GNSS数据依据激光雷达进行同步，寻找激光雷达时间戳左右两个时刻的时间数据，使用线性插值的方式进行同步
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    //bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);
    bool valid_odom=PoseData::SyncData(unsynced_odom_, odom_data_buff_, cloud_time);

    // only mark lidar as 'inited' when all the three sensors are synced:
    // 只有当三个传感器都完成同步才将激光雷达标记为初始化
    static bool sensor_inited = false;
    if (!sensor_inited) {
      if (!valid_imu || !valid_gnss||valid_odom)
        {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool DataPretreatFlow::InitCalibration() {
    // lookup imu pose in lidar frame:
    static bool calibration_received = false;
    if (!calibration_received) {
        // if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
        //     calibration_received = true; 
        // }

        Eigen::Matrix4f imu_to_lidar;
        
        imu_to_lidar << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0, 
           0, 0, 0, 1;


        lidar_to_imu_ = imu_to_lidar.inverse();
        calibration_received = true;
    }

    return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (odom_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;

    return true;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_odom_data_ = odom_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_odom_time = current_cloud_data_.time - current_odom_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    //
    // this check assumes the frequency of lidar is 10Hz:
    //
    if (diff_imu_time < -0.05 || diff_gnss_time < -0.05 ||
        diff_odom_time < -0.05)
      {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_odom_time > 0.05) {
        odom_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    odom_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() {
    // a. get reference pose:
    // 获得gnss在lidar坐标系下的位姿矩阵
    gnss_pose_ = Eigen::Matrix4f::Identity();
    // get position from GNSS
    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    // get orientation from IMU:
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    // this is lidar pose in GNSS/map frame:
    gnss_pose_ *= lidar_to_imu_;

    current_velocity_data_.time = current_odom_data_.time;
    current_velocity_data_.linear_velocity.x = current_odom_data_.vel.x();
    current_velocity_data_.linear_velocity.y = current_odom_data_.vel.y();
    current_velocity_data_.linear_velocity.z = current_odom_data_.vel.z();
    current_velocity_data_.angular_velocity.x = current_imu_data_.angular_velocity.x;
    current_velocity_data_.angular_velocity.y = current_imu_data_.angular_velocity.y;
    current_velocity_data_.angular_velocity.z = current_imu_data_.angular_velocity.z;
    
    // b. set synced pos vel
    // 获得位置和速度数据
    pos_vel_.pos.x() = current_gnss_data_.local_E;
    pos_vel_.pos.y() = current_gnss_data_.local_N;
    pos_vel_.pos.z() = current_gnss_data_.local_U;

    pos_vel_.vel.x() = current_velocity_data_.linear_velocity.x;
    pos_vel_.vel.y() = current_velocity_data_.linear_velocity.y;
    pos_vel_.vel.z() = current_velocity_data_.linear_velocity.z;

    // c. motion compensation for lidar measurements:
    // 获取雷达此时的运动数据并进行畸变校正操作
    current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    imu_pub_ptr_->Publish(current_imu_data_, current_cloud_data_.time);

    pos_vel_pub_ptr_->Publish(pos_vel_, current_cloud_data_.time);
    
    //
    // this synced odometry has the following info:
    //
    // a. lidar frame's pose in map
    // b. lidar frame's velocity 
    gnss_pub_ptr_->Publish(gnss_pose_, current_velocity_data_, current_cloud_data_.time);

    
    return true;
}
}