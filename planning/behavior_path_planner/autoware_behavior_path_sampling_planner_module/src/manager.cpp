// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/behavior_path_sampling_planner_module/manager.hpp"

#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

// 初始化采样规划器模块管理器
void SamplingPlannerModuleManager::init(rclcpp::Node * node)
{
  // 初始化管理器接口
  initInterface(node, {""});

  // 定义采样规划器参数
  SamplingPlannerParameters p{};
  {
    std::string ns{"constraints.hard"};
    // 声明并获取硬约束参数
    p.max_curvature = node->declare_parameter<double>(ns + ".max_curvature"); // 最大曲率
    p.min_curvature = node->declare_parameter<double>(ns + ".min_curvature"); // 最小曲率
    ns = std::string{"constraints.soft"};
    // 声明并获取软约束参数
    p.lateral_deviation_weight =
      node->declare_parameter<double>(ns + ".lateral_deviation_weight");       // 横向偏差权重 [[未使用]] 待删除？
    p.length_weight = node->declare_parameter<double>(ns + ".length_weight");  // 路径长度权重 [[未使用]] 待删除？
    p.curvature_weight =
      node->declare_parameter<double>(ns + ".curvature_weight");  // 曲率权重 [[未使用]] 待删除？
    p.weights = node->declare_parameter<std::vector<double>>(ns + ".weights");   // 权重向量
  }
  {
    std::string ns{"sampling"};
    // 声明并获取采样参数
    p.enable_frenet = node->declare_parameter<bool>(ns + ".enable_frenet"); // 是否启用Frenet坐标系
    p.enable_bezier = node->declare_parameter<bool>(
      ns + ".enable_bezier");  // 是否启用贝塞尔曲线 [[未使用]] 未来会使用
    p.resolution = node->declare_parameter<double>(ns + ".resolution"); // 采样分辨率
    p.previous_path_reuse_points_nb =
      node->declare_parameter<int>(ns + ".previous_path_reuse_points_nb");  // 重用前一路径的点数
    p.nb_target_lateral_positions =
      node->declare_parameter<int>(ns + ".nb_target_lateral_positions");  // 目标横向位置数量
    p.target_lengths = node->declare_parameter<std::vector<double>>(ns + ".target_lengths");  // 目标长度向量
    p.target_lateral_positions =
      node->declare_parameter<std::vector<double>>(ns + ".target_lateral_positions"); // 目标横向位置向量
    ns += ".frenet";
    p.target_lateral_velocities =
      node->declare_parameter<std::vector<double>>(ns + ".target_lateral_velocities");  // 目标横向速度向量
    p.target_lateral_accelerations =
      node->declare_parameter<std::vector<double>>(ns + ".target_lateral_accelerations"); // 目标横向加速度向量
  }
  {
    std::string ns{"preprocessing"};
    // 声明并获取预处理参数
    p.force_zero_deviation = node->declare_parameter<bool>(
      ns + ".force_zero_initial_deviation");  // 是否强制初始横向偏差为零 [[未使用]] 未来会使用
    p.force_zero_heading = node->declare_parameter<bool>(
      ns + ".force_zero_initial_heading");  // 是否强制初始航向为零 [[未使用]] 未来会使用
    p.smooth_reference = node->declare_parameter<bool>(
      ns + ".smooth_reference_trajectory");  // 是否平滑参考轨迹 [[未使用]] 未来会使用
  }

  // 将参数存储到共享指针中
  parameters_ = std::make_shared<SamplingPlannerParameters>(p);
}

// 更新模块参数
void SamplingPlannerModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto & p = parameters_;

  {
    std::string ns{"constraints.hard"};
    // 更新硬约束参数
    update_param<double>(parameters, ns + ".max_curvature", p->max_curvature);  // 更新最大曲率
    update_param<double>(parameters, ns + ".min_curvature", p->min_curvature);  // 更新最小曲率
    ns = std::string{"constraints.soft"};
    // 更新软约束参数
    update_param<double>(parameters, ns + ".lateral_deviation_weight", p->lateral_deviation_weight);  // 更新横向偏差权重
    update_param<double>(parameters, ns + ".length_weight", p->length_weight);  // 更新路径长度权重
    update_param<double>(parameters, ns + ".curvature_weight", p->curvature_weight);  // 更新曲率权重
    update_param<std::vector<double>>(parameters, ns + ".weights", p->weights); // 更新权重向量
  }
  {
    std::string ns{"sampling"};
    // 更新采样参数
    update_param<bool>(parameters, ns + ".enable_frenet", p->enable_frenet);  // 更新是否启用Frenet坐标系
    update_param<bool>(parameters, ns + ".enable_bezier", p->enable_bezier);  // 更新是否启用贝塞尔曲线
    update_param<double>(parameters, ns + ".resolution", p->resolution);  // 更新采样分辨率

    update_param<int>(
      parameters, ns + ".previous_path_reuse_points_nb", p->previous_path_reuse_points_nb); // 更新重用前一路径的点数

    update_param<int>(
      parameters, ns + ".nb_target_lateral_positions", p->nb_target_lateral_positions); // 更新目标横向位置数量
    update_param<std::vector<double>>(parameters, ns + ".target_lengths", p->target_lengths); // 更新目标长度向量

    update_param<std::vector<double>>(
      parameters, ns + ".target_lateral_positions", p->target_lateral_positions); // 更新目标横向位置向量

    ns += ".frenet";
    update_param<std::vector<double>>(
      parameters, ns + ".target_lateral_velocities", p->target_lateral_velocities); // 更新目标横向速度向量

    update_param<std::vector<double>>(
      parameters, ns + ".target_lateral_accelerations", p->target_lateral_accelerations); // 更新目标横向加速度向量
  }
  {
    std::string ns{"preprocessing"};
    // 更新预处理参数
    update_param<bool>(parameters, ns + ".force_zero_initial_deviation", p->force_zero_deviation);  // 更新是否强制初始横向偏差为零
    update_param<bool>(parameters, ns + ".force_zero_initial_heading", p->force_zero_heading);  // 更新是否强制初始航向为零
    update_param<bool>(parameters, ns + ".smooth_reference_trajectory", p->smooth_reference); // 更新是否平滑参考轨迹
  }

  // 遍历所有观察者，更新模块参数
  std::for_each(observers_.begin(), observers_.end(), [&](const auto & observer) {
    if (!observer.expired()) {
      const auto sampling_planner_ptr =
        std::dynamic_pointer_cast<SamplingPlannerModule>(observer.lock());
      if (sampling_planner_ptr) {
        sampling_planner_ptr->updateModuleParams(p);
      }
    }
  });
}

}  // namespace autoware::behavior_path_planner

// 插件导出宏，用于将管理器类注册为插件
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::SamplingPlannerModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
