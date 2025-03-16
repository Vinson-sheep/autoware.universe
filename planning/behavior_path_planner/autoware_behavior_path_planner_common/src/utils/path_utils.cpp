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

#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/interpolation/spline_interpolation.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils
{

/**
 * @brief 计算路径上每个点的弧长，从起点到终点。
 * @param path 路径数据
 * @param start 起始点索引
 * @param end 结束点索引
 * @param offset 弧长偏移量
 * @return 返回从起点到终点的弧长数组
 */
std::vector<double> calcPathArcLengthArray(
  const PathWithLaneId & path, const size_t start, const size_t end, const double offset)
{
  const auto bounded_start = std::max(start, size_t{0});
  const auto bounded_end = std::min(end, path.points.size());
  std::vector<double> out;
  out.reserve(bounded_end - bounded_start);

  double sum = offset;
  out.push_back(sum);

  for (size_t i = bounded_start + 1; i < bounded_end; ++i) {
    sum += autoware_utils::calc_distance2d(path.points.at(i).point, path.points.at(i - 1).point);
    out.push_back(sum);
  }
  return out;
}

/**
 * @brief 使用样条曲线对路径进行重采样。
 * @param path 原始路径
 * @param interval 重采样间隔
 * @param keep_input_points 是否保留输入点
 * @param target_section 目标区间
 * @return 返回重采样后的路径
 */
PathWithLaneId resamplePathWithSpline(
  const PathWithLaneId & path, const double interval, const bool keep_input_points,
  const std::pair<double, double> target_section)
{
  // 安全校验
  if (path.points.size() < 2) {
    return path;
  }

  // 复制path到transformed_path
  std::vector<autoware_planning_msgs::msg::PathPoint> transformed_path(path.points.size());
  for (size_t i = 0; i < path.points.size(); ++i) {
    transformed_path.at(i) = path.points.at(i).point;
  }

  // 定义函数：从vec中寻找与x相同的元素
  const auto find_almost_same_values =
    [&](const std::vector<double> & vec, double x) -> std::optional<std::vector<size_t>> {
    constexpr double epsilon = 0.2;
    const auto is_close = [&](double v, double x) { return std::abs(v - x) < epsilon; };

    std::vector<size_t> indices;
    if (vec.empty()) {
      return std::nullopt;
    }

    for (size_t i = 0; i < vec.size(); ++i) {
      if (is_close(vec[i], x)) {
        indices.push_back(i);
      }
    }

    if (indices.empty()) {
      return std::nullopt;
    }

    return indices;
  };

  // 
  // Get lane ids that are not duplicated

  std::vector<double> s_in;
  std::unordered_set<int64_t> unique_lane_ids;
  // 提取s数组
  const auto s_vec = autoware::motion_utils::calcSignedArcLengthPartialSum(
    transformed_path, 0, transformed_path.size());
  // 遍历所有路径点
  for (size_t i = 0; i < path.points.size(); ++i) {
    const double s = s_vec.at(i);
    // 提取路径点的land_ids
    for (const auto & lane_id : path.points.at(i).lane_ids) {
      // 如果不保留原始点，同时land_id已经存在，则跳过
      if (!keep_input_points && (unique_lane_ids.find(lane_id) != unique_lane_ids.end())) {
        continue;
      }

      // 插入lane_id
      unique_lane_ids.insert(lane_id);

      // 插入s
      if (!find_almost_same_values(s_in, s)) {
        s_in.push_back(s);
      }
    }
  }

  std::vector<double> s_out = s_in;

  // s进行等距采样
  // sampling from interval distance
  const auto start_s = std::max(target_section.first, 0.0);
  const auto end_s = std::min(target_section.second, s_vec.back());
  for (double s = start_s; s < end_s; s += interval) {
    if (!find_almost_same_values(s_out, s)) {
      s_out.push_back(s);
    }
  }
  if (!find_almost_same_values(s_out, end_s)) {
    s_out.push_back(end_s);
  }

  // 计算停止距离，如果停止位置在中间，移除后续s
  // Insert Stop Point
  const auto closest_stop_dist =
    autoware::motion_utils::calcDistanceToForwardStopPoint(transformed_path);
  if (closest_stop_dist) {
    const auto close_indices = find_almost_same_values(s_out, *closest_stop_dist);
    if (close_indices) {
      // Update the smallest index
      s_out.at(close_indices->at(0)) = *closest_stop_dist;

      // Remove the rest of the indices in descending order
      for (size_t i = close_indices->size() - 1; i > 0; --i) {
        s_out.erase(s_out.begin() + close_indices->at(i));
      }
    } else {
      s_out.push_back(*closest_stop_dist);
    }
  }

  // 如果s小于两个点，返回路径
  // spline resample required more than 2 points for yaw angle calculation
  if (s_out.size() < 2) {
    return path;
  }

  // 对s进行排序
  std::sort(s_out.begin(), s_out.end());

  // 对s进行重新采样后返回
  return autoware::motion_utils::resamplePath(path, s_out);
}

/**
 * @brief 根据弧长获取路径点的索引。
 * @param path 路径数据
 * @param target_idx 目标索引
 * @param signed_arc 弧长
 * @return 返回根据弧长计算得到的索引
 */
size_t getIdxByArclength(
  const PathWithLaneId & path, const size_t target_idx, const double signed_arc)
{
  if (path.points.empty()) {
    throw std::runtime_error("[getIdxByArclength] path points must be > 0");
  }

  // 从前往后搜索大于signed_arc的第一个路径点索引
  using autoware_utils::calc_distance2d;
  double sum_length = 0.0;
  if (signed_arc >= 0.0) {
    for (size_t i = target_idx; i < path.points.size() - 1; ++i) {
      const auto next_i = i + 1;
      sum_length += calc_distance2d(path.points.at(i), path.points.at(next_i));
      if (sum_length > signed_arc) {
        return next_i;
      }
    }
    return path.points.size() - 1;
  }
  // 从后往前搜索小于signed_arc的第一个索引
  for (size_t i = target_idx; i > 0; --i) {
    const auto next_i = i - 1;
    sum_length -= calc_distance2d(path.points.at(i), path.points.at(next_i));
    if (sum_length < signed_arc) {
      return next_i;
    }
  }
  return 0;
}

// TODO(murooka) 该函数应替换为 autoware::motion_utils::cropPoints
/**
 * @brief 裁剪路径长度。
 * @param path 路径数据
 * @param target_idx 目标索引
 * @param forward 向前裁剪长度
 * @param backward 向后裁剪长度
 */
void clipPathLength(
  PathWithLaneId & path, const size_t target_idx, const double forward, const double backward)
{
  // 安全检查 
  if (path.points.size() < 3) {
    return;
  }

  // 根据给定的前向和后向距离裁剪路径
  const auto start_idx = utils::getIdxByArclength(path, target_idx, -backward);
  const auto end_idx = utils::getIdxByArclength(path, target_idx, forward);

  const std::vector<PathPointWithLaneId> clipped_points{
    path.points.begin() + start_idx, path.points.begin() + end_idx + 1};

  path.points = clipped_points;
}

/**
 * @brief 将路径点转换为带有车道ID的路径。
 * @param waypoints 路径点数据
 * @param velocity 速度
 * @param lanelets 车道数据
 * @return 返回转换后的路径
 */
PathWithLaneId convertWayPointsToPathWithLaneId(
  const autoware::freespace_planning_algorithms::PlannerWaypoints & waypoints,
  const double velocity, const lanelet::ConstLanelets & lanelets)
{
  PathWithLaneId path;
  path.header = waypoints.header;
  // 遍历所有路径点
  for (size_t i = 0; i < waypoints.waypoints.size(); ++i) {
    const auto & waypoint = waypoints.waypoints.at(i);
    PathPointWithLaneId point{};
    point.point.pose = waypoint.pose.pose;
    // put the lane that contain waypoints in lane_ids.
    bool is_in_lanes = false;
    // 遍历所有lanelet，判断是否在某个lanelet中
    for (const auto & lane : lanelets) {
      if (lanelet::utils::isInLanelet(point.point.pose, lane)) {
        point.lane_ids.push_back(lane.id());
        is_in_lanes = true;
      }
    }
    // 如果不在某个Lanelet中，那么idx直接赋值为上一个点的idx
    // If none of them corresponds, assign the previous lane_ids.
    if (!is_in_lanes && i > 0) {
      point.lane_ids = path.points.at(i - 1).lane_ids;
    }

    // 填写速度
    point.point.longitudinal_velocity_mps = (waypoint.is_back ? -1 : 1) * velocity;
    path.points.push_back(point);
  }
  return path;
}

/**
 * @brief 获取路径中的反向点索引。
 * @param path 路径数据
 * @return 返回反向点的索引列表
 */
std::vector<size_t> getReversingIndices(const PathWithLaneId & path)
{
  std::vector<size_t> indices;

  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    // 如果速度反向，则插入点索引
    if (
      path.points.at(i).point.longitudinal_velocity_mps *
        path.points.at(i + 1).point.longitudinal_velocity_mps <
      0) {
      indices.push_back(i);
    }
  }

  return indices;
}

/**
 * @brief 根据索引将路径分割为多个子路径。
 * @param path 路径数据
 * @param indices 分割索引
 * @return 返回分割后的子路径列表
 */
std::vector<PathWithLaneId> dividePath(
  const PathWithLaneId & path, const std::vector<size_t> & indices)
{
  std::vector<PathWithLaneId> divided_paths;

  // 如果indices为空，直接返回原始路径
  if (indices.empty()) {
    divided_paths.push_back(path);
    return divided_paths;
  }

  // 根据indices对路径进行切割
  for (size_t i = 0; i < indices.size(); ++i) {
    PathWithLaneId divided_path;
    divided_path.header = path.header;
    if (i == 0) {
      divided_path.points.insert(
        divided_path.points.end(), path.points.begin(), path.points.begin() + indices.at(i) + 1);
    } else {
      // include the point at indices.at(i - 1) and indices.at(i)
      divided_path.points.insert(
        divided_path.points.end(), path.points.begin() + indices.at(i - 1),
        path.points.begin() + indices.at(i) + 1);
    }
    divided_paths.push_back(divided_path);
  }

  PathWithLaneId divided_path;
  divided_path.header = path.header;
  divided_path.points.insert(
    divided_path.points.end(), path.points.begin() + indices.back(), path.points.end());
  divided_paths.push_back(divided_path);

  return divided_paths;
}

/**
 * @brief 修正分割路径的速度。
 * @param divided_paths 分割后的路径列表
 */
void correctDividedPathVelocity(std::vector<PathWithLaneId> & divided_paths)
{
  // 遍历所有路径段
  for (auto & path : divided_paths) {

    // 判断是否是前向路径
    const auto is_driving_forward = autoware::motion_utils::isDrivingForward(path.points);
    // If the number of points in the path is less than 2, don't correct the velocity

    // 如果判断失败，直接跳过
    if (!is_driving_forward) {
      continue;
    }

    // 如果是前向路径，则最大速度取正
    if (*is_driving_forward) {
      for (auto & point : path.points) {
        point.point.longitudinal_velocity_mps = std::abs(point.point.longitudinal_velocity_mps);
      }
    } 
    // 如果是后向路径，则最大速度取负
    else {
      for (auto & point : path.points) {
        point.point.longitudinal_velocity_mps = -std::abs(point.point.longitudinal_velocity_mps);
      }
    }
    path.points.back().point.longitudinal_velocity_mps = 0.0;
  }
}

// only two points is supported
std::vector<double> spline_two_points(
  const std::vector<double> & base_s, const std::vector<double> & base_x, const double begin_diff,
  const double end_diff, const std::vector<double> & new_s)
{
  assert(base_s.size() == 2 && base_x.size() == 2);

  const double h = base_s.at(1) - base_s.at(0);

  const double c = begin_diff;
  const double d = base_x.at(0);
  const double a = (end_diff * h - 2 * base_x.at(1) + c * h + 2 * d) / std::pow(h, 3);
  const double b = (3 * base_x.at(1) - end_diff * h - 2 * c * h - 3 * d) / std::pow(h, 2);

  std::vector<double> res;
  for (const auto & s : new_s) {
    const double ds = s - base_s.at(0);
    res.push_back(d + (c + (b + a * ds) * ds) * ds);
  }

  return res;
}

/**
 * @brief 对两个位姿进行插值。
 * @param start_pose 起始位姿
 * @param end_pose 结束位姿
 * @param resample_interval 重采样间隔
 * @return 返回插值后的位姿列表
 */
std::vector<Pose> interpolatePose(
  const Pose & start_pose, const Pose & end_pose, const double resample_interval)
{
  using autoware_utils::calc_azimuth_angle;

  std::vector<Pose> interpolated_poses{};  // output

  // 计算插值距离
  const double distance = autoware_utils::calc_distance2d(start_pose.position, end_pose.position);
  const std::vector<double> base_s{0.0, distance};
  const std::vector<double> base_x{start_pose.position.x, end_pose.position.x};
  const std::vector<double> base_y{start_pose.position.y, end_pose.position.y};
  std::vector<double> new_s;

  // 重采样距离s
  constexpr double eps = 0.3;  // prevent overlapping
  for (double s = eps; s < distance - eps; s += resample_interval) {
    new_s.push_back(s);
  }

  // 插值xyz
  const std::vector<double> interpolated_x = spline_two_points(
    base_s, base_x, std::cos(tf2::getYaw(start_pose.orientation)),
    std::cos(tf2::getYaw(end_pose.orientation)), new_s);
  const std::vector<double> interpolated_y = spline_two_points(
    base_s, base_y, std::sin(tf2::getYaw(start_pose.orientation)),
    std::sin(tf2::getYaw(end_pose.orientation)), new_s);
  for (size_t i = 0; i < interpolated_x.size(); ++i) {
    Pose pose{};
    pose.position.x = interpolated_x.at(i);
    pose.position.y = interpolated_y.at(i);
    pose.position.z = end_pose.position.z;
    interpolated_poses.push_back(pose);
  }

  // 插值yaw
  // insert orientation
  for (size_t i = 0; i < interpolated_poses.size(); ++i) {
    const double yaw = calc_azimuth_angle(
      interpolated_poses.at(i).position, i < interpolated_poses.size() - 1
                                           ? interpolated_poses.at(i + 1).position
                                           : end_pose.position);
    interpolated_poses.at(i).orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  }

  return interpolated_poses;
}

/**
 * @brief 获取未偏移的车辆位姿。
 * @param ego_pose 车辆位姿
 * @param prev_path 之前的路径
 * @return 返回未偏移的车辆位姿
 */
Pose getUnshiftedEgoPose(const Pose & ego_pose, const ShiftedPath & prev_path)
{
  // 安全检查
  if (prev_path.path.points.empty()) {
    return ego_pose;
  }

  // 获取自车投影点索引
  // un-shifted for current ideal pose
  const auto closest_idx =
    autoware::motion_utils::findNearestIndex(prev_path.path.points, ego_pose.position);

  // 获取插值点
  // NOTE: Considering avoidance by motion, we set unshifted_pose as previous path instead of
  // ego_pose.
  auto unshifted_pose =
    autoware::motion_utils::calcInterpolatedPoint(prev_path.path, ego_pose).point.pose;

  unshifted_pose = autoware_utils::calc_offset_pose(
    unshifted_pose, 0.0, -prev_path.shift_length.at(closest_idx), 0.0);
  unshifted_pose.orientation = ego_pose.orientation;

  return unshifted_pose;
}

// TODO(Horibe) 清理函数：util 中有类似的代码。
/**
 * @brief 计算中心线路径。
 * @param planner_data 规划数据
 * @param ref_pose 参考位姿
 * @param longest_dist_to_shift_line 到偏移线的最大距离
 * @param prev_module_path 之前的模块路径
 * @return 返回中心线路径
 */
PathWithLaneId calcCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data, const Pose & ref_pose,
  const double longest_dist_to_shift_line, const std::optional<PathWithLaneId> & prev_module_path)
{
  // 获取规划参数和路由处理器
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;

  PathWithLaneId centerline_path;

  // 计算后向路径长度
  const auto extra_margin = 10.0;  // Since distance does not consider arclength, but just line.
  const auto backward_length =
    std::max(p.backward_path_length, longest_dist_to_shift_line + extra_margin);

  RCLCPP_DEBUG(
    rclcpp::get_logger("path_utils"),
    "p.backward_path_length = %f, longest_dist_to_shift_line = %f, backward_length = %f",
    p.backward_path_length, longest_dist_to_shift_line, backward_length);

  // 获取当前车道
  const lanelet::ConstLanelets current_lanes = [&]() {
    if (!prev_module_path) {
      return utils::calcLaneAroundPose(
        route_handler, ref_pose, p.forward_path_length, backward_length);
    }
    return utils::getCurrentLanesFromPath(*prev_module_path, planner_data);
  }();
  centerline_path = utils::getCenterLinePath(
    *route_handler, current_lanes, ref_pose, backward_length, p.forward_path_length, p);

  centerline_path.header = route_handler->getRouteHeader();

  return centerline_path;
}

/**
 * @brief 合并两条路径。
 * @param path1 第一条路径
 * @param path2 第二条路径
 * @return 返回合并后的路径
 */
PathWithLaneId combinePath(const PathWithLaneId & path1, const PathWithLaneId & path2)
{
  // 安全性检查
  if (path1.points.empty()) {
    return path2;
  }
  if (path2.points.empty()) {
    return path1;
  }

  // 插入path1
  PathWithLaneId path{};
  path.points.insert(path.points.end(), path1.points.begin(), path1.points.end());

  // 插入path2
  // skip overlapping point
  path.points.insert(path.points.end(), next(path2.points.begin()), path2.points.end());

  // 移除重叠点 （如何做到？）
  PathWithLaneId filtered_path = path;
  filtered_path.points = autoware::motion_utils::removeOverlapPoints(filtered_path.points);
  return filtered_path;
}

/**
 * @brief 获取参考路径。
 * @param current_lane 当前车道
 * @param planner_data 规划数据
 * @return 返回参考路径
 */
BehaviorModuleOutput getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  PathWithLaneId reference_path{};

  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_odometry->pose.pose;
  const auto p = planner_data->parameters;

  // Set header
  // 提取参考线header
  reference_path.header = route_handler->getRouteHeader();

  // calculate path with backward margin to avoid end points' instability by spline interpolation
  constexpr double extra_margin = 10.0;
  const double backward_length = p.backward_path_length + extra_margin;
  // 提取范围内的lanelets
  const auto current_lanes_with_backward_margin =
    route_handler->getLaneletSequence(current_lane, backward_length, p.forward_path_length);
  // 提取自车最近的中心线位姿
  const auto no_shift_pose =
    lanelet::utils::getClosestCenterPose(current_lane, current_pose.position);
  // 获取中心线路径
  reference_path = getCenterLinePath(
    *route_handler, current_lanes_with_backward_margin, no_shift_pose, backward_length,
    p.forward_path_length, p);

  // 对路径进行裁剪
  // clip backward length
  // NOTE: In order to keep backward_path_length at least, resampling interval is added to the
  // backward.
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      reference_path.points, no_shift_pose, p.ego_nearest_dist_threshold,
      p.ego_nearest_yaw_threshold);
  reference_path.points = autoware::motion_utils::cropPoints(
    reference_path.points, no_shift_pose.position, current_seg_idx, p.forward_path_length,
    p.backward_path_length + p.input_path_interval);

  // 从参考线中提取lanelets
  const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);
  // 从参考线中提取lane
  const auto drivable_lanes = generateDrivableLanes(drivable_lanelets);

  const auto & dp = planner_data->drivable_area_expansion_parameters;

  // 对lanes进行裁剪
  const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);
  // 对lanes进行膨胀
  const auto expanded_lanes = expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  // 填写output并返回
  BehaviorModuleOutput output;
  output.path = reference_path;
  output.reference_path = reference_path;
  output.drivable_area_info.drivable_lanes = drivable_lanes;

  return output;
}

/**
 * @brief 创建目标周围的路径。
 * @param planner_data 规划数据
 * @return 返回目标周围的路径
 */
BehaviorModuleOutput createGoalAroundPath(const std::shared_ptr<const PlannerData> & planner_data)
{
  BehaviorModuleOutput output;

  // 获取目标位姿
  const auto & route_handler = planner_data->route_handler;
  const auto & modified_goal = planner_data->prev_modified_goal;

  const Pose goal_pose = modified_goal ? modified_goal->pose : route_handler->getGoalPose();

  // 尝试从目标点的位姿获取目标车道，优先使用路肩车道，如果没有则使用普通车道
  lanelet::ConstLanelet goal_lane;
  const auto shoulder_goal_lanes = route_handler->getShoulderLaneletsAtPose(goal_pose);
  if (!shoulder_goal_lanes.empty()) goal_lane = shoulder_goal_lanes.front();
  const auto is_failed_getting_lanelet =
    shoulder_goal_lanes.empty() && !route_handler->getGoalLanelet(&goal_lane);
  if (is_failed_getting_lanelet) {
    return output;
  }

  // 生成局部路径
  constexpr double backward_length = 1.0;
  const auto arc_coord = lanelet::utils::getArcCoordinates({goal_lane}, goal_pose);
  const double s_start = std::max(arc_coord.length - backward_length, 0.0);
  const double s_end = arc_coord.length;

  // 获取Lane的中心线
  auto reference_path = route_handler->getCenterLinePath({goal_lane}, s_start, s_end);

  // 提取相关lanelet
  const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);

  // 提取所有lane
  const auto drivable_lanes = generateDrivableLanes(drivable_lanelets);

  const auto & dp = planner_data->drivable_area_expansion_parameters;

  // 缩减lane
  const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);

  // 膨胀lane
  const auto expanded_lanes = expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  // 目标速度为0，表示刹车
  // Insert zero velocity to each point in the path.
  for (auto & point : reference_path.points) {
    point.point.longitudinal_velocity_mps = 0.0;
  }

  // 填写输出
  output.path = reference_path;
  output.reference_path = reference_path;
  output.drivable_area_info.drivable_lanes = drivable_lanes;

  return output;
}

}  // namespace autoware::behavior_path_planner::utils
