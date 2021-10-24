/*
 * @Description: 点云畸变补偿
 * @Author: Ren Qian
 * @Date: 2020-02-25 14:39:00
 */
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "glog/logging.h"

namespace lidar_localization {
void DistortionAdjust::SetMotionInfo(float scan_period, VelocityData velocity_data) {
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}

bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) {
    CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));
    output_cloud_ptr.reset(new CloudData::CLOUD());

    float orientation_space = 2.0 * M_PI;   // 2π
    float delete_space = 5.0 * M_PI / 180.0;    // 5π/180
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);  // 开始角度

    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ()); // 旋转向量，以角度和轴赋值
    Eigen::Matrix3f rotate_matrix = t_V.matrix();                       // 获得该旋转向量下的旋转矩阵 
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();         // 获得位姿矩阵，平移部分为0
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);   // 根据位姿矩阵变换原始点云

    velocity_ = rotate_matrix * velocity_;          // 计算变换后的速度矩阵
    angular_rate_ = rotate_matrix * angular_rate_;  // 计算变换后的加速度矩阵

    // 对每一个点进行操作
    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);
        if (orientation < 0.0)
            orientation += 2.0 * M_PI;
        
        if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
            continue;

        // 计算当前点的实际扫描时间
        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;

        // 对每一个点进行去畸变处理
        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
        // 对当前点进行旋转和平移处理
        Eigen::Vector3f rotated_point = current_matrix * origin_point;
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;

        // 将新的点保存下来
        CloudData::POINT point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    // 将之前根据第一个点的旋转向量变换后的点云位置给变换回来
    // 重新拉回到lidar坐标系
    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    return true;
}

Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) {
    Eigen::Vector3f angle = angular_rate_ * real_time;
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;
    return t_V.matrix();
}
} // namespace lidar_localization