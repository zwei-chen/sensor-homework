/*
 * @Description: 点云匹配模块的基类
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:25:11
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class RegistrationInterface {
  public:
    /**
     * @brief 默认析构函数 
     *        在实现多态时，当用基类操作派生类，在析构时防止只析构基类而不析构派生类的状况发生。
     */    
    virtual ~RegistrationInterface() = default;

    /**
     * @brief 传入局部地图
     * @param input_target 局部地图
     */    
    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;

    /**
     * @brief 
     * @param 
     * @return 
     */    
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          CloudData::CLOUD_PTR& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
};
} 

#endif