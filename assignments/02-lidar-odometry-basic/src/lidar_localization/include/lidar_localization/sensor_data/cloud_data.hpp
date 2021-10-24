/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_localization {
/**
 * 点云类型数据
 */
class CloudData {
  public:
    using POINT = pcl::PointXYZ;            // 点
    using CLOUD = pcl::PointCloud<POINT>;   // 点云
    using CLOUD_PTR = CLOUD::Ptr;           // 点云指针，即点云

  public:
    /**
     * @brief 构造函数
     *        通过接收到的点云信息构建该点云
     */
    CloudData()
      :cloud_ptr(new CLOUD()) {
    }

  public:
    double time = 0.0;                      // 时间戳
    CLOUD_PTR cloud_ptr;                    // 点云
};
}

#endif