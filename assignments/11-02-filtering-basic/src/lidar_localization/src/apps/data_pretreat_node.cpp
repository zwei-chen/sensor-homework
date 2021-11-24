/*
 * @Description: 数据预处理的node文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    // 初始化goolgle日志
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";   // 设置日志输出目录
    FLAGS_alsologtostderr = 1;  // 将所有信息打印到终端

    // ros节点的初始化
    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;

    // 从参数服务器上读取数据，并赋值，如果从参数服务器上无法读取数据，则使用默认数据赋值
    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    // subscribe to
    // a. raw Velodyne measurement
    // b. raw GNSS/IMU measurement
    // publish
    // a. undistorted Velodyne measurement
    // b. lidar pose in map frame
    // 该类主要有以下几个功能
    // 接受者
    // Velodyne的激光雷达测量数据
    // OXTS惯导的GNSS和IMU测量数据
    // 发布者
    // 发布去畸变的Velodyne激光雷达测量数据
    // 发布雷达在地图坐标系下的位姿
    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh, cloud_topic);

    // pre-process lidar point cloud at 100Hz:
    // 预处理雷达点云以100hz的速率进行处理
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}