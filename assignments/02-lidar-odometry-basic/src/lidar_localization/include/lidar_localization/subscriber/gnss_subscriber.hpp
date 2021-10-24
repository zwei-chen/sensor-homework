/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-03-31 12:58:10
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "lidar_localization/sensor_data/gnss_data.hpp"

namespace lidar_localization {
class GNSSSubscriber {
  public:
    /**
     * @brief 构造函数，初始化
     * @param nh ros句柄
     * @param topic_name 话题名称
     * @param buff_size 缓冲区长度
     */  
    GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    /**
     * @brief 默认构造函数
     */    
    GNSSSubscriber() = default;
    
    /**
     * @brief 在deque_gnss_data位置尾部插入new_gnss_data_所有数据，并清空new_gnss_data_
     * @param deque_gnss_data 在里程计里存放gnss数据信息的队列
     */ 
    void ParseData(std::deque<GNSSData>& deque_gnss_data);

  private:
    /**
     * @brief 对接收到的ros类型gnss数据转化为文件内定义gnss数据类型，并添加到new_gnss_data_队列中
     * @param nav_sat_fix_ptr ros类型的gnss数据
     */    
    void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

  private:
    ros::NodeHandle nh_;                    // ros句柄
    ros::Subscriber subscriber_;            // ros接收者

    std::deque<GNSSData> new_gnss_data_;    // 存放GNSS数据的队列
};
}
#endif
