/*
 * @Description: 从点云中截取一个立方体部分
 * @Author: Ren Qian
 * @Date: 2020-03-04 20:09:37
 */

#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_BOX_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_BOX_FILTER_HPP_

#include <pcl/filters/crop_box.h>
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class BoxFilter: public CloudFilterInterface {
  public:
    /**
     * @brief 构造函数
     *        从配置文件读取立方体滤波器参数
     *        并构建立方体滤波器
     *        用于过滤指定立方体内的点
     * @param node 配置文件
     */    
    BoxFilter(YAML::Node node);

    /**
     * @brief  默认构造函数
     */    
    BoxFilter() = default;

    /**
     * @brief 对点云进行立方体滤波器操作
     *        该函数有override关键字，用作提醒将会重写这个函数
     * @param input_cloud_ptr 滤波前的点云
     * @param filtered_cloud_ptr 滤波后的点云
     * @return 不管怎么样，都返回ture，可作为完成该函数的功能的标识
     */   
    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

    void SetSize(std::vector<float> size);
    void SetOrigin(std::vector<float> origin);
    std::vector<float> GetEdge();

  private:
    void CalculateEdge();

  private:
    pcl::CropBox<CloudData::POINT> pcl_box_filter_; // 立方体滤波器，用于过滤指定立方体内的点

    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;
};
}

#endif 