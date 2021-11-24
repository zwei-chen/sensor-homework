/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-28 18:50:16
 */
#include "lidar_localization/sensor_data/pose_data.hpp"
#include <iostream>

namespace lidar_localization {

Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    return q;
}

void PoseData::GetVelocityData(VelocityData &velocity_data) const {
    velocity_data.time = time;

    velocity_data.linear_velocity.x = vel.v.x();
    velocity_data.linear_velocity.y = vel.v.y();
    velocity_data.linear_velocity.z = vel.v.z();

    velocity_data.angular_velocity.x = vel.w.x();
    velocity_data.angular_velocity.y = vel.w.y();
    velocity_data.angular_velocity.z = vel.w.z();
}

bool PoseData::SyncData(std::deque<PoseData>& UnsyncedData, std::deque<PoseData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值

    // ！！！寻找需要同步时刻左右两帧速度数据
    // 若时间差异大于0.2s，说明数据有丢失，时间离得太远不适合做差值   
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 2) {
            UnsyncedData.pop_front();
            return false;
        }
        if (UnsyncedData.at(1).time - sync_time > 2) {
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    PoseData front_data = UnsyncedData.at(0);
    PoseData back_data = UnsyncedData.at(1);
    PoseData synced_data;

    // 使用线性插值对线速度和角速度进行解算
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.vel.v.x() = front_data.vel.v.x() * front_scale + back_data.vel.v.x() * back_scale;
    synced_data.vel.v.y() = front_data.vel.v.y() * front_scale + back_data.vel.v.y() * back_scale;
    synced_data.vel.v.z() = front_data.vel.v.z() * front_scale + back_data.vel.v.z() * back_scale;
    synced_data.vel.w.x() = front_data.vel.w.x() * front_scale + back_data.vel.w.x() * back_scale;
    synced_data.vel.w.y() = front_data.vel.w.y() * front_scale + back_data.vel.w.y() * back_scale;
    synced_data.vel.w.z() = front_data.vel.w.z() * front_scale + back_data.vel.w.z() * back_scale;
    
    //synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
    //synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
    //synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
    //synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    //synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    //synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}

}