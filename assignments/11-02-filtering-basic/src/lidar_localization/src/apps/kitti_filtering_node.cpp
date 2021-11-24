/*
 * @Description: IMU-lidar fusion for localization
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include <ros/ros.h>

#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/filtering/kitti_filtering_flow.hpp"

#include <lidar_localization/saveOdometry.h>

#include "glog/logging.h"

using namespace lidar_localization;

bool _need_save_odometry = false;

/**
 * @brief 作为rosService接受请求，并在主程序的下一次循环中进行保存里程计的操作
 * @param request rosService请求
 * @param response rosService相应
 * @return 相应成功的标识
 */
bool SaveOdometryCB(saveOdometry::Request &request,
                    saveOdometry::Response &response) {
  _need_save_odometry = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char *argv[]) {
  // 初始化goolgle日志
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";   // 设置日志输出目录
  FLAGS_alsologtostderr = 1;                  // 将所有信息打印到终端

  // ros节点的初始化
  ros::init(argc, argv, "kitti_filtering_node");
  ros::NodeHandle nh;

  std::shared_ptr<KITTIFilteringFlow> kitti_filtering_flow_ptr =
      std::make_shared<KITTIFilteringFlow>(nh);
  
  // ros服务器，此服务器为接收响应并进行保存里程计操作
  ros::ServiceServer service =
      nh.advertiseService("save_odometry", SaveOdometryCB);

  ros::Rate rate(100);  
  while (ros::ok()) {
    ros::spinOnce();

    kitti_filtering_flow_ptr->Run();

    // save odometry estimations for evo evaluation:
    // 保存里程计信息便于后续进行evo评估
    if (_need_save_odometry && kitti_filtering_flow_ptr->SaveOdometry()) {
      _need_save_odometry = false;
    }

    rate.sleep();
  }

  return 0;
}