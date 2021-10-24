/*
 * @Description: front-end node, lidar odometry basic
 * @Author: Ge Yao
 * @Date: 2021-04-03
 */

#include <memory>

#include <ros/ros.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "glog/logging.h"

#include "lidar_localization/front_end/front_end_flow.hpp"

#include <lidar_localization/saveMap.h>

using namespace lidar_localization;

bool save_map = false;  // 是否进行保存地图功能的标识

/**
 * @brief ros服务器的回调函数，每当获得一个请求时，将save_map置1，并返回需要得到了响应
 *        save_map置1后，当里程计更新成功后，将会保存一次地图
 * @return 回调函数成功调用后返回给客户端信号
 */ 
bool save_map_callback(
    saveMap::Request &request, 
    saveMap::Response &response
) {
    save_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    // 初始化goolgle日志
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";   // 设置日志输出目录
    FLAGS_alsologtostderr = 1;  // 将所有信息打印到终端

    // ros节点的初始化
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    // register service save_map:
    // 初始化服务器，接收请求并返回响应
    // ！！！savemap部分，先不看
    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);

    // register front end processing workflow:
    // 初始化前段里程计
    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    // process rate: 10Hz
    // 设置处理频率
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        // 进入里程计
        front_end_flow_ptr->Run();

        // 保存地图用
        // 其是基于ros服务器的通讯机制，每当获得一个需求时，将会将save_true置1，便会运行一次该功能
        if (save_map) {
            front_end_flow_ptr->SaveMap();
            front_end_flow_ptr->PublishGlobalMap();

            save_map = false;   // 每成功保存一次地图，将其置为false
        }

        rate.sleep();
    }

    return EXIT_SUCCESS;
}