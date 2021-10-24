/*
 * @Description: 在ros中发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
/**
 *  点云发布模块
 */
class CloudPublisher {
  public:
    /**
     * @brief 构造函数，初始化点云发布者
     * @param nh ros句柄
     * @param topic_name 话题名称
     * @param buff_size 缓冲区长度
     * @param frame_id 坐标系
     */ 
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);

    /**
     * @brief 默认构造函数
     */ 
    CloudPublisher() = default;

    /**
     * @brief 以指定时间，发布点云数据
     * @param cloud_ptr_input 点云数据
     * @param time 时间
     */ 
    void Publish(CloudData::CLOUD_PTR& cloud_ptr_input, double time);

    /**
     * @brief 以当前ros时间，发布点云数据
     * @param cloud_ptr_input 点云数据
     */ 
    void Publish(CloudData::CLOUD_PTR& cloud_ptr_input);

    /**
     * @brief 查看是否有节点订阅所需话题
     * @return 有节点订阅，则返回true
     */    
    bool HasSubscribers();
  
  private:
    /**
     * @brief 发布点云数据
     * @param cloud_ptr_input 点云数据
     * @param time 时间
     */ 
    void PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time);

  private:
    ros::NodeHandle nh_;        // ros句柄
    ros::Publisher publisher_;  // ros发布者
    std::string frame_id_;      // 将要发布数据的坐标系
};
} 
#endif