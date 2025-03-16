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

#include "autoware/planning_topic_converter/path_to_trajectory.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <vector>

namespace autoware::planning_topic_converter
{
namespace
{

// 将 PathPoint 转换为 TrajectoryPoint
TrajectoryPoint convertToTrajectoryPoint(const PathPoint & point)
{
  TrajectoryPoint traj_point;
  traj_point.pose = autoware_utils::get_pose(point);  // 获取姿态
  traj_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps; // 设置纵向速度
  traj_point.lateral_velocity_mps = point.lateral_velocity_mps; // 设置横向速度
  traj_point.heading_rate_rps = point.heading_rate_rps; // 设置航向角速度
  return traj_point;
}

// 将 PathPoint 的向量转换为 TrajectoryPoint 的向量
std::vector<TrajectoryPoint> convertToTrajectoryPoints(const std::vector<PathPoint> & points)
{
  std::vector<TrajectoryPoint> traj_points;
  for (const auto & point : points) {
    const auto traj_point = convertToTrajectoryPoint(point);  // 转换单个点
    traj_points.push_back(traj_point);  // 添加到结果向量
  }
  return traj_points;
}
}  // namespace

// 构造函数，初始化节点
PathToTrajectory::PathToTrajectory(const rclcpp::NodeOptions & options)
: ConverterBase("path_to_trajectory_converter", options)
{
}

// 处理接收到的路径消息
void PathToTrajectory::process(const Path::ConstSharedPtr msg)
{
  const auto trajectory_points = convertToTrajectoryPoints(msg->points);  // 转换路径点为轨迹点
  const auto output = autoware::motion_utils::convertToTrajectory(trajectory_points, msg->header);  // 转换为轨迹消息
  pub_->publish(output);  // 发布轨迹消息
}

}  // namespace autoware::planning_topic_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning_topic_converter::PathToTrajectory)
