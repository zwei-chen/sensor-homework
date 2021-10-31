
/**
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lio-mapping>
* or <https://sites.google.com/view/lio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by hyye on 5/4/18.
//

/// adapted from VINS-mono

#ifndef LIO_MARGINALIZATIONFACTOR_H_
#define LIO_MARGINALIZATIONFACTOR_H_

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include <unordered_map>
#include <pthread.h>

#include "utils/CircularBuffer.h"
#include "utils/Twist.h"
#include "utils/common_ros.h"
#include "utils/TicToc.h"
#include "utils/math_utils.h"
#include "utils/geometry_utils.h"

namespace lio {

const int NUM_THREADS = 4;

struct ResidualBlockInfo {
  ResidualBlockInfo(ceres::CostFunction *_cost_function,
                    ceres::LossFunction *_loss_function,
                    std::vector<double *> _parameter_blocks,
                    std::vector<int> _drop_set)
      : cost_function(_cost_function),
        loss_function(_loss_function),
        parameter_blocks(_parameter_blocks),
        drop_set(_drop_set) {}

  /**
   * @brief 计算损失函数相应的残差和雅克比
   * @param void
   * @return void
   */  
  void Evaluate();

  ceres::CostFunction *cost_function;     // 代价函数
  ceres::LossFunction *loss_function;     // 损失函数
  std::vector<double *> parameter_blocks; // 参数块，包含所有的相关参数
  std::vector<int> drop_set;              // 需要边缘化掉的参数

  double **raw_jacobians;                 // 雅克比
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;  // 雅克比矩阵
  Eigen::VectorXd residuals;              // 残差

  int localSize(int size) {
    return size == 7 ? 6 : size;
  }
};

struct ThreadsStruct {
  std::vector<ResidualBlockInfo *> sub_factors;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  std::unordered_map<long, int> parameter_block_size; //global size
  std::unordered_map<long, int> parameter_block_idx; //local size
};

// TAG: 边缘化相关类
class MarginalizationInfo {
 public:

  /**
   * @brief 析构函数,清空相应的变量
   * @param void
   * @return void 
   */  
  ~MarginalizationInfo();

  /**
   * @brief size==7，则返回6，否则返回7
   * @param size 大小
   * @return void
   */  
  int LocalSize(int size) const;

  /**
   * @brief 
   * @param size 大小
   * @return  
   */  
  int GlobalSize(int size) const;

  /**
   * @brief step1:添加边缘化需要的信息参数
   *        添加ResidualBlockInfo的同时，核心变量parameter_block_size被赋值
   * @param residual_block_info
   * @return 
   */  
  void AddResidualBlockInfo(ResidualBlockInfo *residual_block_info);

  /**
   * @brief step2:边缘化的预处理
   *        计算每个因子对应的变量(parameter_blocks)、误差项(residuals)、
   *        雅可比矩阵( jacobians)，并把变量数值放到parameter_block_data
   * @param void
   * @return void
   */
  void PreMarginalize();

  /**
   * @brief step3:边缘化过程
   * @param void
   * @return void
   */  
  void Marginalize();

  
  std::vector<double *> GetParameterBlocks(std::unordered_map<long, double *> &addr_shift);

  std::vector<ResidualBlockInfo *> factors;     // 残差块
  int m, n; // m 需要marg掉的变量的总维度
            // n 需要保留变量的总维度 
  std::unordered_map<long, int> parameter_block_size; //global size 每个变量的维度
  int sum_block_size;
  std::unordered_map<long, int> parameter_block_idx; //local size 每个变量在H矩阵中的索引
  std::unordered_map<long, double *> parameter_block_data; // 每个变量的数据

  std::vector<int> keep_block_size; //global size 需要保留变量的大小
  std::vector<int> keep_block_idx;  //local size 需要保留变量在H矩阵中的索引
  std::vector<double *> keep_block_data;  //需要保留的变量的数据

  Eigen::MatrixXd linearized_jacobians;
  Eigen::VectorXd linearized_residuals;
  const double eps = 1e-8;
};

class MarginalizationFactor : public ceres::CostFunction {
 public:
  MarginalizationFactor(MarginalizationInfo* _marginalization_info);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

  MarginalizationInfo* marginalization_info;
};

}

#endif //LIO_MARGINALIZATIONFACTOR_H_
