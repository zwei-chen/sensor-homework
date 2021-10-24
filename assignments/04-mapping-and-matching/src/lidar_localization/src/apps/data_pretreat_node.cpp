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

    // 从参数服务器获取参数值给变量，若参数服务器中没有参数值，则将默认值赋予给变量
    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    // subscribe to
    // a. raw Velodyne measurement
    // b. raw GNSS/IMU measurement
    // publish
    // a. undistorted Velodyne measurement
    // b. lidar pose in map frame
    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh, cloud_topic);

    // pre-process lidar point cloud at 100Hz:
    // 设置雷达点云处理频率为100hz
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        // 进入主程序
        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}