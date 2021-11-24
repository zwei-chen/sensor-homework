/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR&  cloud_ptr_input, double time) {
    ros::Time ros_time(time);
    PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR&  cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}

void CloudPublisher::PublishData(CloudData::CLOUD_PTR&  cloud_ptr_input, ros::Time time) {
    // ros中通讯中使用的数据类型
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    // 数据类型转换，将pcl类型数据转化为ros类型    
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    // 添加时间和坐标系信息
    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    // 根据是否有节点订阅所需话题，决定是否发布对应话题
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization