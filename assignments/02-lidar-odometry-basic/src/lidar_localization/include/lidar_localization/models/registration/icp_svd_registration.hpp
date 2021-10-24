/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_SVD_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_SVD_REGISTRATION_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"

#include <pcl/kdtree/kdtree_flann.h>

namespace lidar_localization {

// 从虚类继承过来
class ICPSVDRegistration: public RegistrationInterface {
  public:
    /**
     * @brief 构造函数
     *        载入ICP算法所使用的参数
     *        同时初始化kd数存储结构
     * @param node 配置文件
     */ 
    ICPSVDRegistration(const YAML::Node& node);

    /**
     * @brief 构造函数
     *        构造该类的时候将参数赋予给类成员
     *        同时初始化kd数存储结构
     * @param max_corr_dist 临近点对的距离阈值
     * @param trans_eps 最大容差，一旦两次转换矩阵小于 trans_eps  退出迭代
     * @param euc_fitness_eps
     * @param max_iter 最大迭代次数
     */    
    ICPSVDRegistration(
      float max_corr_dist, 
      float trans_eps, 
      float euc_fitness_eps, 
      int max_iter
    );

    /**
     * @brief 对虚函数的重写 
     *        保存点云
     *        将传入的点云构造成kd树结构
     *        其针对的是局部地图点云的处理
     *        主要针对插入关键帧之后局部地图的更新
     * @param input_target 稀疏化后的局部地图点云
     * @return 不管怎么样，都返回ture，可作为完成该函数的功能的标识
     */ 
    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    
    /**
     * @brief 特征匹配
     * @param input_source  滤波后的点云
     * @param predict_pose  GNSS的预测值
     * @param result_cloud_ptr  点云匹配结果
     * @param result_pose 当前帧的位姿
     */ 
    bool ScanMatch(
      const CloudData::CLOUD_PTR& input_source, 
      const Eigen::Matrix4f& predict_pose, 
      CloudData::CLOUD_PTR& result_cloud_ptr,
      Eigen::Matrix4f& result_pose
    ) override;
  
  private:
    /**
     * @brief 将载入的参数赋予给类成员
     *        同时输出参数信息
     * @param max_corr_dist 临近点对的距离阈值
     * @param trans_eps 最大容差，一旦两次转换矩阵小于 trans_eps  退出迭代
     * @param euc_fitness_eps
     * @param max_iter 最大迭代次数
     * @return 不管怎么样，都返回ture，可作为完成该函数的功能的标识
     */    
    bool SetRegistrationParam(
      float max_corr_dist, 
      float trans_eps, 
      float euc_fitness_eps, 
      int max_iter
    );

  private:
    /**
     * @brief 通过kdtree进行点云配对
     * @param input_sorce 需要匹配的点云
     * @param xs 配对上的点
     * @param ys 配对上的点
     * @return 点云匹配数量
     */ 
    size_t GetCorrespondence(
      const CloudData::CLOUD_PTR &input_source, 
      std::vector<Eigen::Vector3f> &xs,
      std::vector<Eigen::Vector3f> &ys
    );

    /**
     * @brief 基于SVD分解求解ICP
     *        传参的x和y是配对好的点
     * @param xs 配对上的点
     * @param ys 配对上的点
     * @param transformation_ 两者之间的相对位姿变换
     */ 
    void GetTransform(
      const std::vector<Eigen::Vector3f> &xs,
      const std::vector<Eigen::Vector3f> &ys,
      Eigen::Matrix4f &transformation_ 
    );

    /**
     * @brief 判断旋转和平移的变换是否超过最大容差
     * @param transformation
     * @param trans_eps 最大容差
     * @return 如果超过最大容差，则返回true
     */    
    bool IsSignificant(
      const Eigen::Matrix4f &transformation,
      const float trans_eps
    );

    float max_corr_dist_;      // 临近点对的距离阈值
    float trans_eps_;          // 最大容差，一旦两次转换矩阵小于 trans_eps  退出迭代
    float euc_fitness_eps_;    // 没有使用
    int max_iter_;            // 最大迭代次数

    CloudData::CLOUD_PTR input_target_; // 需要匹配的目标点云
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr input_target_kdtree_;  // kd树存储
    CloudData::CLOUD_PTR input_source_; // 源点云

    Eigen::Matrix4f transformation_;  //所估计的姿态
};

}

#endif