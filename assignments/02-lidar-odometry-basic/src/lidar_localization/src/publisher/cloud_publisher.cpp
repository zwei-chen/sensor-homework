/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "lidar_localization/publisher/cloud_publisher.hpp"

namespace lidar_localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               size_t buff_size,
                               std::string frame_id)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR  cloud_ptr_input) {
    // ros中通讯中使用的数据类型
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    // 数据类型转换，将pcl类型数据转化为ros类型    
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
    // 添加时间和坐标系信息
    cloud_ptr_output->header.stamp = ros::Time::now();
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

} // namespace data_output