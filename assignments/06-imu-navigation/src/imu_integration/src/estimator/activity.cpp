/*
 * @Description: IMU integration activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#include <cmath>

#include "imu_integration/estimator/activity.hpp"
#include "glog/logging.h"

#include <fstream>
#include <boost/filesystem.hpp>

#define MID_VALUE_MODE 0

namespace imu_integration {

namespace estimator {

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

Activity::Activity(void) 
    : private_nh_("~"), 
    initialized_(false),
    // gravity acceleration:
    G_(0, 0, -9.81),
    // angular velocity bias:
    angular_vel_bias_(0.0, 0.0, 0.0),
    // linear acceleration bias:
    linear_acc_bias_(0.0, 0.0, 0.0)
{}

void Activity::Init(void) {
    // parse IMU config:
    private_nh_.param("imu/topic_name", imu_config_.topic_name, std::string("/sim/sensor/imu"));
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(private_nh_, imu_config_.topic_name, 1000000);

    // a. gravity constant:
    private_nh_.param("imu/gravity/x", imu_config_.gravity.x,  0.0);
    private_nh_.param("imu/gravity/y", imu_config_.gravity.y,  0.0);
    private_nh_.param("imu/gravity/z", imu_config_.gravity.z, -9.81);
    G_.x() = imu_config_.gravity.x;
    G_.y() = imu_config_.gravity.y;
    G_.z() = imu_config_.gravity.z;

    // b. angular velocity bias:
    private_nh_.param("imu/bias/angular_velocity/x", imu_config_.bias.angular_velocity.x,  0.0);
    private_nh_.param("imu/bias/angular_velocity/y", imu_config_.bias.angular_velocity.y,  0.0);
    private_nh_.param("imu/bias/angular_velocity/z", imu_config_.bias.angular_velocity.z,  0.0);
    angular_vel_bias_.x() = imu_config_.bias.angular_velocity.x;
    angular_vel_bias_.y() = imu_config_.bias.angular_velocity.y;
    angular_vel_bias_.z() = imu_config_.bias.angular_velocity.z;

    // c. linear acceleration bias:
    private_nh_.param("imu/bias/linear_acceleration/x", imu_config_.bias.linear_acceleration.x,  0.0);
    private_nh_.param("imu/bias/linear_acceleration/y", imu_config_.bias.linear_acceleration.y,  0.0);
    private_nh_.param("imu/bias/linear_acceleration/z", imu_config_.bias.linear_acceleration.z,  0.0);
    linear_acc_bias_.x() = imu_config_.bias.linear_acceleration.x;
    linear_acc_bias_.y() = imu_config_.bias.linear_acceleration.y;
    linear_acc_bias_.z() = imu_config_.bias.linear_acceleration.z;

    // parse odom config:
    private_nh_.param("pose/frame_id", odom_config_.frame_id, std::string("inertial"));
    private_nh_.param("pose/topic_name/ground_truth", odom_config_.topic_name.ground_truth, std::string("/pose/ground_truth"));
    private_nh_.param("pose/topic_name/estimation", odom_config_.topic_name.estimation, std::string("/pose/estimation"));

    odom_ground_truth_sub_ptr = std::make_shared<OdomSubscriber>(private_nh_, odom_config_.topic_name.ground_truth, 1000000);
    odom_estimation_pub_ = private_nh_.advertise<nav_msgs::Odometry>(odom_config_.topic_name.estimation, 500);
}

bool Activity::Run(void) {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (UpdatePose()) {
            PublishPose();
            SaveTrajectoryKitti();
        }
    }

    return true;
}

bool Activity::ReadData(void) {
    // fetch IMU measurements into buffer:
    imu_sub_ptr_->ParseData(imu_data_buff_);

    if (static_cast<size_t>(0) == imu_data_buff_.size())
        return false;

    if (!initialized_) {
        odom_ground_truth_sub_ptr->ParseData(odom_data_buff_);

        if (static_cast<size_t>(0) == odom_data_buff_.size())
            return false;
    }

    return true;
}

bool Activity::HasData(void) {
    if (imu_data_buff_.size() < static_cast<size_t>(3))
        return false;

    if (
        !initialized_ && 
        static_cast<size_t>(0) == odom_data_buff_.size()
    ) {
        return false;
    }

    return true;
}

bool Activity::UpdatePose(void) {
    if (!initialized_) {
        // use the latest measurement for initialization:
        // 使用最后一次的里程计进行初始化：
        OdomData odom_data = odom_data_buff_.back();
        IMUData imu_data = imu_data_buff_.back();

        pose_ = odom_data.pose;
        vel_ = odom_data.vel;

        initialized_ = true;

        odom_data_buff_.clear();
        imu_data_buff_.clear();

        // keep the latest IMU measurement for mid-value integration:
        // 使用最新的IMU测量值进行中值法解算
        imu_data_buff_.push_back(imu_data);
    } else {
        //
        // TODO: implement your estimation here
        // 估计的解算过程
        //
        // get deltas:
        // 获取角度
        Eigen::Vector3d deltas = Eigen::Vector3d::Zero();
        GetAngularDelta(1, 0, deltas);

        // update orientation:
        // 更新旋转
        Eigen::Matrix3d R_curr = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d R_prev = Eigen::Matrix3d::Identity();
        UpdateOrientation(deltas, R_curr, R_prev);

        // get velocity delta:
        // 获得速度
        double delta_t = 0;
        Eigen::Vector3d velocity_delta = Eigen::Vector3d::Zero();
        GetVelocityDelta(1, 0, R_curr, R_prev, delta_t, velocity_delta);

        // update position:
        // 更新位置
        UpdatePosition(delta_t, velocity_delta);

        // move forward -- 
        // NOTE: this is NOT fixed. you should update your buffer according to the method of your choice:
        // 移除最早的数据
        imu_data_buff_.pop_front();
    }
    
    return true;
}

bool Activity::PublishPose() {
    // a. set header:
    message_odom_.header.stamp = ros::Time::now();
    message_odom_.header.frame_id = odom_config_.frame_id;
    
    // b. set child frame id:
    message_odom_.child_frame_id = odom_config_.frame_id;

    // b. set orientation:
    Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));
    message_odom_.pose.pose.orientation.x = q.x();
    message_odom_.pose.pose.orientation.y = q.y();
    message_odom_.pose.pose.orientation.z = q.z();
    message_odom_.pose.pose.orientation.w = q.w();

    // c. set position:
    Eigen::Vector3d t = pose_.block<3, 1>(0, 3);
    message_odom_.pose.pose.position.x = t.x();
    message_odom_.pose.pose.position.y = t.y();
    message_odom_.pose.pose.position.z = t.z();  

    // d. set velocity:
    message_odom_.twist.twist.linear.x = vel_.x();
    message_odom_.twist.twist.linear.y = vel_.y();
    message_odom_.twist.twist.linear.z = vel_.z(); 

    odom_estimation_pub_.publish(message_odom_);

    return true;
}

/**
 * @brief  get unbiased angular velocity in body frame
 * @param  angular_vel, angular velocity measurement
 * @return unbiased angular velocity in body frame
 */
inline Eigen::Vector3d Activity::GetUnbiasedAngularVel(const Eigen::Vector3d &angular_vel) {
    return angular_vel - angular_vel_bias_;
}

/**
 * @brief  get unbiased linear acceleration in navigation frame
 * @param  linear_acc, linear acceleration measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased linear acceleration in navigation frame
 */
inline Eigen::Vector3d Activity::GetUnbiasedLinearAcc(
    const Eigen::Vector3d &linear_acc,
    const Eigen::Matrix3d &R
) {
    return R*(linear_acc - linear_acc_bias_) - G_;
}

/**
 * @brief  get angular delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  angular_delta, angular delta output
 * @return true if success false otherwise
 */
bool Activity::GetAngularDelta(
    const size_t index_curr, const size_t index_prev,
    Eigen::Vector3d &angular_delta
) {
    //
    // TODO: this could be a helper routine for your own implementation
    //
    if (
        index_curr <= index_prev ||
        imu_data_buff_.size() <= index_curr
    ) {
        return false;
    }

    const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

    double delta_t = imu_data_curr.time - imu_data_prev.time;

    Eigen::Vector3d angular_vel_curr = GetUnbiasedAngularVel(imu_data_curr.angular_velocity);
    Eigen::Vector3d angular_vel_prev = GetUnbiasedAngularVel(imu_data_prev.angular_velocity);

    // mid-value
    if(MID_VALUE_MODE)
        angular_delta = 0.5*delta_t*(angular_vel_curr + angular_vel_prev);
    // euler's-value
    else
        angular_delta = delta_t*angular_vel_prev;

    return true;
}

/**
 * @brief  get velocity delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  R_curr, corresponding orientation of current imu measurement
 * @param  R_prev, corresponding orientation of previous imu measurement
 * @param  velocity_delta, velocity delta output
 * @return true if success false otherwise
 */
bool Activity::GetVelocityDelta(
    const size_t index_curr, const size_t index_prev,
    const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, 
    double &delta_t, Eigen::Vector3d &velocity_delta
) {
    //
    // TODO: this could be a helper routine for your own implementation
    //
    if (
        index_curr <= index_prev ||
        imu_data_buff_.size() <= index_curr
    ) {
        return false;
    }

    const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

    delta_t = imu_data_curr.time - imu_data_prev.time;

    Eigen::Vector3d linear_acc_curr = GetUnbiasedLinearAcc(imu_data_curr.linear_acceleration, R_curr);
    Eigen::Vector3d linear_acc_prev = GetUnbiasedLinearAcc(imu_data_prev.linear_acceleration, R_prev);
    
    // mid-value
    if(MID_VALUE_MODE)
        velocity_delta = 0.5*delta_t*(linear_acc_curr + linear_acc_prev);
    // euler's-value
    else
        velocity_delta = delta_t* linear_acc_prev;


    return true;
}

/**
 * @brief  update orientation with effective rotation angular_delta
 * @param  angular_delta, effective rotation
 * @param  R_curr, current orientation
 * @param  R_prev, previous orientation
 * @return void
 */
void Activity::UpdateOrientation(
    const Eigen::Vector3d &angular_delta,
    Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev
) {
    //
    // TODO: this could be a helper routine for your own implementation
    //
    // magnitude:
    double angular_delta_mag = angular_delta.norm();
    // direction:
    Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

    // build delta q:
    double angular_delta_cos = cos(angular_delta_mag/2.0);
    double angular_delta_sin = sin(angular_delta_mag/2.0);
    Eigen::Quaterniond dq(
        angular_delta_cos, 
        angular_delta_sin*angular_delta_dir.x(), 
        angular_delta_sin*angular_delta_dir.y(), 
        angular_delta_sin*angular_delta_dir.z()
    );
    Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));
    
    // update:
    q = q*dq;
    
    // write back:
    R_prev = pose_.block<3, 3>(0, 0);
    pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    R_curr = pose_.block<3, 3>(0, 0);
}

/**
 * @brief  update orientation with effective velocity change velocity_delta
 * @param  delta_t, timestamp delta 
 * @param  velocity_delta, effective velocity change
 * @return void
 */
void Activity::UpdatePosition(const double &delta_t, const Eigen::Vector3d &velocity_delta) {
    //
    // TODO: this could be a helper routine for your own implementation
    //
    pose_.block<3, 1>(0, 3) += delta_t*vel_ + 0.5*delta_t*velocity_delta;
    vel_ += velocity_delta;
}

bool Activity::SaveTrajectoryKitti() {
    static std::ofstream ground_truth, estimation;
    static bool is_file_created = false;
    std::string WORK_SPACE_PATH="/workspace/assignments/06-imu-navigation/src/imu_integration";
    
    if (!is_file_created) {
        if (!CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
            return false;
        //if (!CreateFile(ground_truth, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt"))
            //return false;
        if (!CreateFile(estimation, WORK_SPACE_PATH + "/slam_data/trajectory/estimation.txt"))
            return false;
        is_file_created = true;
    }

    // 存入数据
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            //ground_truth << odom_pose(i, j);
            estimation << pose_(i, j);
            if (i == 2 && j == 3) {
                //ground_truth << std::endl;
                estimation << std::endl;
            } else {
                //ground_truth << " ";
                estimation << " ";
            }
        }
    }

    return true;
}

} // namespace estimator

} // namespace imu_integration