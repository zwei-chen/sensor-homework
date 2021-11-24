/*
 * @Description: 点云滤波模块的接口
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:29:50
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
/**
 * 点云滤波模块的接口
 */ 
class CloudFilterInterface {
  public:
    /**
     * @brief 默认析构函数 
     *        在实现多态时，当用基类操作派生类，在析构时防止只析构基类而不析构派生类的状况发生。
     */    
    virtual ~CloudFilterInterface() = default;

    /**
     * @brief 对点云进行滤波器操作
     *        =0表示该函数为纯虚函数
     *        1. 纯虚函数没有函数体；
     *        2. 一个类里如果包含 ＝0 的纯虚函数，那么这个类就是一个抽象类
     *        3. 抽象类不能具体实例化（不能创建它的对象），而只能由它去派生子类
     *        4. 在派生类中对此函数提供定义后，它才能具备函数的功能，可被调用。
     * @param input_cloud_ptr 滤波前的点云
     * @param filtered_cloud_ptr 滤波后的点云
     * @return 由子类决定
     */    
    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};
}

#endif