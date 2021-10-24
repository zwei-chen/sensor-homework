/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_

#include <deque>

#include <Eigen/Dense>

#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"

#include "lidar_localization/models/registration/icp_registration.hpp"
#include "lidar_localization/models/registration/icp_svd_registration.hpp"

#include "lidar_localization/models/registration/ndt_registration.hpp"

#include "lidar_localization/models/registration/sicp/scip_registration.hpp"

namespace lidar_localization {
class FrontEnd {
  public:
    /**
     * 帧，包含位姿和点云数据
     */ 
    struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    /**
     * @brief 构造函数
     *        并从前端里程计的配置文件进行初始化
     */ 
    FrontEnd();

    /**
     * @brief 从前端里程计的配置文件进行初始化
     *        创建点云和关键帧的保存文件夹
     *        初始化位姿求解方法类型
     *        初始化局部地图、当前帧和rviz显示的滤波方法类型
     * @return 不管怎么样，都返回ture，可作为完成该函数的功能的标识
     */ 
    bool InitWithConfig();

    /**
     * @brief 对雷达里程计进行更新
     *        流程依次为点云预处理、点云匹配、关键帧插入的判断
     * @param cloud_data 当前帧的点云
     * @param cloud_pose 当前帧的位姿
     * @return 完成里程计更新则为true
     */
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);

    /**
     * @brief 设置初始位姿，通常是GNSS位姿
     * @param init_pose GNSS位姿
     */ 
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

    /**
     * @brief 通过关键帧数据构建地图并保存
     * @return 不管怎么样，都返回ture，可作为完成该函数的功能的标识
     */    
    bool SaveMap();

    /**
     * @brief 使用rviz显示的滤波方式对局部地图进行滤波
     * @param local_map_ptr 局部地图点云
     * @return 如果有局部地图则为true
     *         局部地图的标识在局部地图初始化后置true
     */    
    bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);

    /**
     * @brief 用rviz显示的滤波方式对全局地图进行滤波
     *        同时情况全局地图
     * @param global_map_ptr 全局地图点云 
     * @return 如果有全局地图则为true
     *         全局地图的标识在保存全局地图中置true
     */    
    bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);

    /**
     * @brief 使用rviz显示的滤波方式对当前帧进行滤波
     * @param current_scan_ptr 当前帧的点云
     * @return 不管怎么样，都返回ture，可作为完成该函数的功能的标识
     */    
    bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

  private:
    /**
     * @brief 读取关键帧的距离和局部地图的数量
     *        实际好像没有使用？？？
     * @param config_node 配置文件
     * @return 不管怎么样，都返回ture，可作为完成该函数的功能的标识
     */    
    bool InitParam(const YAML::Node& config_node);

    /**
     * @brief 初始化点云数据存放路径和关键帧存放路径的文件夹
     * @param config_node 配置文件
     * @return 文件夹创建均成功则为true，有一个创建不成功则为false
     */ 
    bool InitDataPath(const YAML::Node& config_node);

    /**
     * @brief 初始化点云匹配后的求解方法
     * @param registration_ptr 位姿求解方法，点云匹配模块的基类
     * @param config_node 配置文件
     * @return 求解方法获取成功则返回true，没有获得配置文件相应的求解方法则为false
     */ 
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);

    /**
     * @brief 初始化不同情况下使用的点云滤波方法
     *        目前只有voxel_filter这种滤波方法
     * @param filter_user 滤波方法使用者的名称
     * @param filter_ptr 点云滤波方法的指针
     * @param config_node 配置文件
     * @return 滤波方法获取成功则返回true，没有获得配置文件相应的滤波方法则为false
     */ 
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);

    /**
     * @brief 当关键帧生成时，进行关键帧的传入
     *        将关键帧保存在本地
     *        同时维护局部地图
     * @param new_key_frame 新添加的关键帧
     * @return 不管怎么样，都返回ture，可作为完成该函数的功能的标识
     */    
    bool UpdateWithNewFrame(const Frame& new_key_frame);

  private:
    std::string data_path_ = "";  // 数据存放路径
                                                                  // ？？？滤波器的的具体实现方式暂时没看
    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;      // 当前帧点云滤波方法
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;  // 滑窗地图点云滤波方法
    std::shared_ptr<CloudFilterInterface> display_filter_ptr_;    // rviz 实时显示点云时滤波方法
    std::shared_ptr<RegistrationInterface> registration_ptr_;     // 位姿求解器

    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;          // 是否有局部地图的标识
    bool has_new_global_map_ = false;         // 是否有全局地图的标识
    CloudData::CLOUD_PTR local_map_ptr_;      // 局部地图
    CloudData::CLOUD_PTR global_map_ptr_;     // 结果地图，只在SaveMap中构建，构建后并销毁
    CloudData::CLOUD_PTR result_cloud_ptr_;   // 点云匹配结果
    Frame current_frame_;                     // 当前帧

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity(); // 在雷达里程计更新之前，由Gnss信号赋予，

    float key_frame_distance_ = 2.0;  // 关键帧之间的最小距离
    int local_frame_num_ = 20;        // 局部地图大小
};

}

#endif