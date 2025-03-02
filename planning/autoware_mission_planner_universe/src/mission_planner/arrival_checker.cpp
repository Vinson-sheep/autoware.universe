// Copyright 2022 TIER IV, Inc.
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

#include "arrival_checker.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <tf2/utils.h>

namespace autoware::mission_planner_universe
{

// 构造函数，初始化到达检查器
ArrivalChecker::ArrivalChecker(rclcpp::Node * node) : vehicle_stop_checker_(node)
{
  // 从参数服务器读取角度阈值（单位为度），并转换为弧度
  const double angle_deg = node->declare_parameter<double>("arrival_check_angle_deg");
  angle_ = autoware_utils::deg2rad(angle_deg);
  // 从参数服务器读取距离阈值
  distance_ = node->declare_parameter<double>("arrival_check_distance");
  // 从参数服务器读取持续时间阈值
  duration_ = node->declare_parameter<double>("arrival_check_duration");
}

// 清除目标点
void ArrivalChecker::set_goal()
{
  // 清除当前目标点
  // Ignore the modified goal after the route is cleared.
  goal_with_uuid_ = std::nullopt;
}

// 设置目标点
void ArrivalChecker::set_goal(const PoseWithUuidStamped & goal)
{
  // 设置新的目标点，并忽略与之前目标点 UUID 不匹配的修改
  // Ignore the modified goal for the previous route using uuid.
  goal_with_uuid_ = goal;
}

// 检查车辆是否到达目标点
bool ArrivalChecker::is_arrived(const PoseStamped & pose) const
{
  // 如果没有目标点，则返回未到达
  if (!goal_with_uuid_) {
    return false;
  }
  const auto goal = goal_with_uuid_.value();

  // 检查目标点和车辆位姿的坐标系是否一致
  // Check frame id
  if (goal.header.frame_id != pose.header.frame_id) {
    return false;
  }

  // 检查车辆与目标点的距离是否小于阈值
  // Check distance.
  if (distance_ < autoware_utils::calc_distance2d(pose.pose, goal.pose)) {
    return false;
  }

  // 检查车辆与目标点的角度差是否小于阈值
  // Check angle.
  const double yaw_pose = tf2::getYaw(pose.pose.orientation); // 获取车辆的偏航角
  const double yaw_goal = tf2::getYaw(goal.pose.orientation); // 获取目标点的偏航角
  const double yaw_diff = autoware_utils::normalize_radian(yaw_pose - yaw_goal);  // 计算角度差
  if (angle_ < std::fabs(yaw_diff)) {
    return false; // 如果角度差大于阈值，则返回未到达
  }

  // 检查车辆是否已停止
  // Check vehicle stopped.
  return vehicle_stop_checker_.isVehicleStopped(duration_);
}

}  // namespace autoware::mission_planner_universe
