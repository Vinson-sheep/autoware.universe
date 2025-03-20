// Copyright 2020 Tier IV, Inc.
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

#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <vector>

namespace autoware::behavior_velocity_planner
{

// 使用样条插值对路径进行重采样
bool splineInterpolate(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & input, const double interval,
  autoware_internal_planning_msgs::msg::PathWithLaneId & output, const rclcpp::Logger & logger)
{
  if (input.points.size() < 2) {
    RCLCPP_DEBUG(logger, "Do not interpolate because path size is 1."); // 如果路径点少于2个，不进行插值
    return false;
  }

  output = autoware::motion_utils::resamplePath(input, interval, false, true, true, false); // 使用重采样工具进行插值

  return true;
}

/*
 * 使用固定间隔对路径进行插值。
 * 为了正确继承计划速度点的位置，输入路径中现有点的位置将保留在插值路径中。
 * 速度通过零阶保持进行插值，即插值点的速度是输入“子路径”中最接近点的速度，
 * 该子路径由插值点之前的点组成。
 */
autoware_planning_msgs::msg::Path interpolatePath(
  const autoware_planning_msgs::msg::Path & path, const double length, const double interval)
{
  const auto logger{rclcpp::get_logger("behavior_velocity_planner").get_child("path_utilization")}; // 获取日志记录器

  const double epsilon = 0.01;  // 定义一个小的阈值
  std::vector<double> s_in; // 用于存储输入路径的累积弧长
  if (2000 < path.points.size()) {
    RCLCPP_WARN(
      logger, "because path size is too large, calculation cost is high. size is %d.",
      (int)path.points.size()); // 如果路径点过多，警告计算成本高
  }
  if (path.points.size() < 2) {
    RCLCPP_WARN(logger, "Do not interpolate because path size is 1.");  // 如果路径点少于2个，不进行插值
    return path;
  }

  double path_len = std::min(length, autoware::motion_utils::calcArcLength(path.points)); // 计算路径长度
  {
    std::vector<double> x;  // 存储x坐标
    std::vector<double> y;  // 存储y坐标
    std::vector<double> z;  // 存储z坐标
    std::vector<double> v;  // 存储速度
    double s = 0.0; // 累积弧长
    for (size_t idx = 0; idx < path.points.size(); ++idx) {
      const auto path_point = path.points.at(idx);  // 获取路径点
      x.push_back(path_point.pose.position.x);  // 存储x坐标
      y.push_back(path_point.pose.position.y);  // 存储y坐标
      z.push_back(path_point.pose.position.z);  // 存储z坐标
      v.push_back(path_point.longitudinal_velocity_mps);  // 存储速度
      if (idx != 0) {
        const auto path_point_prev = path.points.at(idx - 1); // 获取前一个路径点
        s += autoware_utils::calc_distance2d(path_point_prev.pose, path_point.pose);  // 累加弧长
      }
      if (s > path_len) {
        break;  // 如果累积弧长超过路径长度，停止
      }
      s_in.push_back(s);  // 存储累积弧长
    }

    // 更新路径长度
    path_len = std::min(path_len, s_in.back());

    // 检查终端点
    if (std::fabs(s_in.back() - path_len) < epsilon) {
      s_in.back() = path_len; // 如果最后一个点接近路径长度，直接设置为路径长度
    } else {
      s_in.push_back(path_len); // 否则，添加路径长度作为最后一个点
    }
  }

  // Calculate query points
  // Use all values of s_in to inherit the velocity-planned point, and add some points for
  // interpolation with a constant interval if the point is not closed to the original s_in points.
  // (because if this interval is very short, the interpolation will be less accurate and may fail.)

  // 计算查询点
  // 使用所有s_in值继承速度计划点，并在不接近原始s_in点的情况下，以固定间隔添加一些点进行插值
  std::vector<double> s_out = s_in; // 初始化输出累积弧长

  const auto has_almost_same_value = [&](const auto & vec, const auto x) {
    if (vec.empty()) return false;  // 如果向量为空，返回false
    const auto has_close = [&](const auto v) { return std::abs(v - x) < epsilon; }; // 检查是否有接近的值
    return std::find_if(vec.begin(), vec.end(), has_close) != vec.end();  // 查找是否有接近的值
  };
  for (double s = 0.0; s < path_len; s += interval) {
    if (!has_almost_same_value(s_out, s)) {
      s_out.push_back(s); // 如果没有接近的值，添加到输出累积弧长
    }
  }

  std::sort(s_out.begin(), s_out.end());  // 对输出累积弧长排序

  if (s_out.empty()) {
    RCLCPP_WARN(logger, "Do not interpolate because s_out is empty.");  // 如果输出累积弧长为空，警告
    return path;
  }

  return autoware::motion_utils::resamplePath(path, s_out);
}

// 过滤掉路径中距离过近的点
autoware_planning_msgs::msg::Path filterLitterPathPoint(
  const autoware_planning_msgs::msg::Path & path)
{
  autoware_planning_msgs::msg::Path filtered_path;  // 初始化过滤后的路径

  const double epsilon = 0.01;  // 定义一个小的阈值
  size_t latest_id = 0; // 初始化最新的点索引
  for (size_t i = 0; i < path.points.size(); ++i) {
    double dist = 0.0;  // 初始化距离
    if (i != 0) {
      const double x =
        path.points.at(i).pose.position.x - path.points.at(latest_id).pose.position.x;  // 计算x方向的距离
      const double y =
        path.points.at(i).pose.position.y - path.points.at(latest_id).pose.position.y;  // 计算y方向的距离
      dist = std::sqrt(x * x + y * y);  // 计算欧几里得距离
    }
    if (i == 0 || epsilon < dist /*init*/) {
      latest_id = i;  // 如果是第一个点或距离大于阈值，更新最新的点索引
      filtered_path.points.push_back(path.points.at(latest_id));  // 添加到过滤后的路径
    } else {
      filtered_path.points.back().longitudinal_velocity_mps = std::min(
        filtered_path.points.back().longitudinal_velocity_mps,
        path.points.at(i).longitudinal_velocity_mps); // 更新速度为最小值
    }
  }

  return filtered_path; // 返回过滤后的路径
}

// 过滤掉路径中速度为零的点
autoware_planning_msgs::msg::Path filterStopPathPoint(
  const autoware_planning_msgs::msg::Path & path)
{
  autoware_planning_msgs::msg::Path filtered_path = path; // 初始化过滤后的路径
  bool found_stop = false;  // 标记是否找到停止点
  for (auto & point : filtered_path.points) {
    if (std::fabs(point.longitudinal_velocity_mps) < 0.01) {
      found_stop = true;  // 如果速度接近零，标记为停止点
    }
    if (found_stop) {
      point.longitudinal_velocity_mps = 0.0;  // 将速度设置为零
    }
  }
  return filtered_path; // 返回过滤后的路径
}
}  // namespace autoware::behavior_velocity_planner
