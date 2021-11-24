/*
 * @Description: voxel filter 模块
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:37:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
/**
 * voxel filter模块
 * 继承至CloudFilterInterface
 */ 
class VoxelFilter: public CloudFilterInterface {
  public:
    /**
     * @brief 构造函数
     *        从配置文件读取体素滤波器参数
     *        并构建pcl体素滤波器
     * @param node 配置文件
     */    
    VoxelFilter(const YAML::Node& node);

    /**
     * @brief 构造函数
     *        从输入参数构建pcl体素滤波器
     * @param leaf_size_x 三维体素栅格的长
     * @param leaf_size_y 三维体素栅格的宽
     * @param leaf_size_z 三维体素栅格的高
     */   
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    /**
     * @brief 对点云进行体素滤波器操作
     *        该函数有override关键字，用作提醒将会重写这个函数
     * @param input_cloud_ptr 滤波前的点云
     * @param filtered_cloud_ptr 滤波后的点云
     * @return 不管怎么样，都返回ture，可作为完成该函数的功能的标识
     */    
    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    /**
     * @brief 从传参构建pcl体素滤波器
     * @param leaf_size_x 三维体素栅格的长
     * @param leaf_size_y 三维体素栅格的宽
     * @param leaf_size_z 三维体素栅格的高
     * @return 不管怎么样，都返回ture，可作为完成该函数的功能的标识
     */ 
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<CloudData::POINT> voxel_filter_; // 体素滤波器，生成三维体素栅格
                                                    // x、y、z为每一个体素的大小
};
}
#endif