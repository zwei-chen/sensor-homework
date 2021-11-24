/*
 * @Description: Subscribe to ROS odometry message
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#include "imu_integration/subscriber/odom_subscriber.hpp"
#include "glog/logging.h"
#include <eigen3/Eigen/src/Geometry/RotationBase.h>

#include <fstream>
#include <boost/filesystem.hpp>

namespace imu_integration {

/**
 * @brief 创建文件，以便后续存入数据
 */ 
bool CreateFile(std::ofstream& ofs, std::string file_path){

    ofs.open(file_path.c_str(), std::ios::app);
    if (!ofs) {
        std::cout << "Can't create file: " << file_path;
        return false;
    }

    return true;

}

/**
 * @brief 创建文件夹，以便后续存入数据
 */ 
bool CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        std::cout << "Can't create directory: " << directory_path;
        return false;
    }
    return true;
}

OdomSubscriber::OdomSubscriber(
  ros::NodeHandle& nh, 
  std::string topic_name, 
  size_t buff_size
) :nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &OdomSubscriber::msg_callback, this);
}

void OdomSubscriber::msg_callback(
  const nav_msgs::OdometryConstPtr& odom_msg_ptr
) {
    buff_mutex_.lock();

    // convert ROS IMU to GeographicLib compatible GNSS message:
    OdomData odom_data;
    odom_data.time = odom_msg_ptr->header.stamp.toSec();

    Eigen::Quaterniond q(
      odom_msg_ptr->pose.pose.orientation.w,
      odom_msg_ptr->pose.pose.orientation.x,
      odom_msg_ptr->pose.pose.orientation.y,
      odom_msg_ptr->pose.pose.orientation.z
    );
    Eigen::Vector3d t(
      odom_msg_ptr->pose.pose.position.x,
      odom_msg_ptr->pose.pose.position.y,
      odom_msg_ptr->pose.pose.position.z      
    );

    odom_data.pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    odom_data.pose.block<3, 1>(0, 3) = t;

    odom_data.vel = Eigen::Vector3d(
      odom_msg_ptr->twist.twist.linear.x,
      odom_msg_ptr->twist.twist.linear.y,
      odom_msg_ptr->twist.twist.linear.z 
    );

    // add new message to buffer:
    odom_data_.push_back(odom_data);
    
    buff_mutex_.unlock();

    static std::ofstream ground_truth;
    static bool is_file_created = false;
    std::string WORK_SPACE_PATH="/workspace/assignments/06-imu-navigation/src/imu_integration";
    
    if (!is_file_created) {
      CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory");
      CreateFile(ground_truth, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt");
      is_file_created = true;
    }
    

    // 解析轨迹数据
    Eigen::Matrix4d odom_pose = odom_data.pose;
    Eigen::Matrix4d trans = Eigen::Matrix4d::Zero();
    trans(0, 3) = odom_pose(0,3);
    trans(1,3) = odom_pose(1,3);
    trans(2,3) = odom_pose(2,3);

    Eigen::Quaterniond odom_q( odom_pose.block<3,3>(0,0) );
    trans.block<3,3>(0,0) = odom_q.toRotationMatrix();    

    // 存入数据
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ground_truth << odom_pose(i, j);
            if (i == 2 && j == 3) {
                ground_truth << std::endl;
            } else {
                ground_truth << " ";
            }
        }
    }

}

void OdomSubscriber::ParseData(
  std::deque<OdomData>& odom_data
) {
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (odom_data_.size() > 0) {
        odom_data.insert(odom_data.end(), odom_data_.begin(), odom_data_.end());
        odom_data_.clear();
    }

    buff_mutex_.unlock();
}

} // namespace imu_integration