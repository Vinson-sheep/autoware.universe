// Copyright 2023 TIER IV, Inc.
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

// #include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include "autoware/motion_utils/trajectory/conversion.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/velocity_smoother/trajectory_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/utils.h>

#include <iostream>
#include <utility>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <memory>

// 用于自动驾驶系统中的路径规划模块。通过对路径进行平滑处理，确保生成的路径在速度和加速度上是平滑的，从而提高车辆行驶的舒适性和安全性。

namespace autoware::behavior_velocity_planner
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using geometry_msgs::msg::Quaternion;
using TrajectoryPointWithIdx = std::pair<TrajectoryPoint, size_t>;

// 平滑路径点，从车辆当前位置开始到路径结束
bool smoothPath(
  const PathWithLaneId & in_path, PathWithLaneId & out_path,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  const geometry_msgs::msg::Pose current_pose = planner_data->current_odometry->pose; // 获取当前车辆姿态
  const double v0 = planner_data->current_velocity->twist.linear.x; // 获取当前车辆速度
  const double a0 = planner_data->current_acceleration->accel.accel.linear.x; // 获取当前车辆加速度
  const auto & external_v_limit = planner_data->external_velocity_limit;  // 获取外部速度限制
  const auto & smoother = planner_data->velocity_smoother_; // 获取速度平滑器

  // 将输入路径转换为轨迹点
  auto trajectory = autoware::motion_utils::convertToTrajectoryPoints<
    autoware_internal_planning_msgs::msg::PathWithLaneId>(in_path);

  // 应用横向加速度滤波
  const auto traj_lateral_acc_filtered = smoother->applyLateralAccelerationFilter(trajectory);

  // 应用转向率限制
  const auto traj_steering_rate_limited =
    smoother->applySteeringRateLimit(traj_lateral_acc_filtered, false);

  // 根据车辆速度重采样轨迹
  auto traj_resampled = smoother->resampleTrajectory(
    traj_steering_rate_limited, v0, current_pose, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold);
  
  // 查找最近的轨迹点
  const size_t traj_resampled_closest =
    autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_resampled, current_pose, planner_data->ego_nearest_dist_threshold,
      planner_data->ego_nearest_yaw_threshold);
  std::vector<TrajectoryPoints> debug_trajectories; // 用于调试的轨迹向量

  // 从最近点裁剪轨迹
  TrajectoryPoints clipped;
  TrajectoryPoints traj_smoothed;
  clipped.insert(
    clipped.end(), traj_resampled.begin() + static_cast<std::ptrdiff_t>(traj_resampled_closest),
    traj_resampled.end());

  // 应用速度平滑
  if (!smoother->apply(v0, a0, clipped, traj_smoothed, debug_trajectories, false)) {
    std::cerr << "[behavior_velocity][trajectory_utils]: failed to smooth" << std::endl;
    return false;
  }

  // 将裁剪后的轨迹与未裁剪部分合并
  traj_smoothed.insert(
    traj_smoothed.begin(), traj_resampled.begin(),
    traj_resampled.begin() + static_cast<std::ptrdiff_t>(traj_resampled_closest));

  // 如果有外部速度限制，应用最大速度限制
  if (external_v_limit) {
    autoware::velocity_smoother::trajectory_utils::applyMaximumVelocityLimit(
      traj_resampled_closest, traj_smoothed.size(), external_v_limit->max_velocity, traj_smoothed);
  }

  // 将平滑后的轨迹点转换为带车道ID的路径
  out_path = autoware::motion_utils::convertToPathWithLaneId<TrajectoryPoints>(traj_smoothed);
  return true;
}

}  // namespace autoware::behavior_velocity_planner
