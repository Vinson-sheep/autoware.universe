// Copyright 2021-2024 TIER IV, Inc.
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

#include "autoware/route_handler/route_handler.hpp"

#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/route_checker.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/path.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <tf2/utils.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::route_handler
{
namespace
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::Path;
using autoware_utils::create_point;
using autoware_utils::create_quaternion_from_yaw;
using geometry_msgs::msg::Pose;
using lanelet::utils::to2D;

// 检查是否存在指定 ID 的车道
bool exists(const std::vector<LaneletPrimitive> & primitives, const int64_t & id)
{
  for (const auto & p : primitives) {
    if (p.id == id) {
      return true;
    }
  }
  return false;
}

// 检查是否存在指定项
bool exists(const lanelet::ConstLanelets & vectors, const lanelet::ConstLanelet & item)
{
  for (const auto & i : vectors) {
    if (i.id() == item.id()) {
      return true;
    }
  }
  return false;
}

// 根据弧长获取 3D 点
lanelet::ConstPoint3d get3DPointFrom2DArcLength(
  const lanelet::ConstLanelets & lanelet_sequence, const double s)
{
  double accumulated_distance2d = 0;
  for (const auto & llt : lanelet_sequence) {
    const auto & centerline = llt.centerline();
    lanelet::ConstPoint3d prev_pt;
    if (!centerline.empty()) {
      prev_pt = centerline.front();
    }
    for (const auto & pt : centerline) {
      const double distance2d = lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
      if (accumulated_distance2d + distance2d > s) {
        const double ratio = (s - accumulated_distance2d) / distance2d;
        const auto interpolated_pt = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
        return lanelet::ConstPoint3d{
          lanelet::InvalId, interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z()};
      }
      accumulated_distance2d += distance2d;
      prev_pt = pt;
    }
  }
  return lanelet::ConstPoint3d{};
}

// 去除路径中的重复点
PathWithLaneId removeOverlappingPoints(const PathWithLaneId & input_path)
{
  PathWithLaneId filtered_path;
  for (const auto & pt : input_path.points) {

    // 插入第一个点
    if (filtered_path.points.empty()) {
      filtered_path.points.push_back(pt);
      continue;
    }

    constexpr double min_dist = 0.001;
    // 如果是重复点
    if (autoware_utils::calc_distance3d(filtered_path.points.back().point, pt.point) < min_dist) {
      // 修改最后一个点的参数
      filtered_path.points.back().lane_ids.push_back(pt.lane_ids.front());
      filtered_path.points.back().point.longitudinal_velocity_mps = std::min(
        pt.point.longitudinal_velocity_mps,
        filtered_path.points.back().point.longitudinal_velocity_mps);
    } else {
      // 插入一个点
      filtered_path.points.push_back(pt);
    }
  }
  // 复制左右边界
  filtered_path.left_bound = input_path.left_bound;
  filtered_path.right_bound = input_path.right_bound;
  return filtered_path;
}

// 将点转换为字符串
std::string toString(const geometry_msgs::msg::Pose & pose)
{
  std::stringstream ss;
  ss << "(" << pose.position.x << ", " << pose.position.y << "," << pose.position.z << ")";
  return ss.str();
}

// 检查两个点是否接近
bool isClose(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, const double epsilon)
{
  return std::abs(p1.x - p2.x) < epsilon && std::abs(p1.y - p2.y) < epsilon;
}

// 将路径点转换为参考点
PiecewiseReferencePoints convertWaypointsToReferencePoints(
  const std::vector<geometry_msgs::msg::Point> & piecewise_waypoints)
{
  PiecewiseReferencePoints piecewise_ref_points;
  for (const auto & piecewise_waypoint : piecewise_waypoints) {
    piecewise_ref_points.push_back(ReferencePoint{true, piecewise_waypoint});
  }
  return piecewise_ref_points;
}

// 检查索引是否在向量范围内
template <typename T>
bool isIndexWithinVector(const std::vector<T> & vec, const int index)
{
  return 0 <= index && index < static_cast<int>(vec.size());
}

// 从向量中移除指定索引的元素
template <typename T>
void removeIndicesFromVector(std::vector<T> & vec, std::vector<size_t> indices)
{
  // 按降序排序索引
  std::sort(indices.begin(), indices.end(), std::greater<int>());

  // 从向量中移除指定索引的元素
  for (const size_t index : indices) {
    vec.erase(vec.begin() + index);
  }
}

// 计算弧坐标
lanelet::ArcCoordinates calcArcCoordinates(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & point)
{
  return lanelet::geometry::toArcCoordinates(
    to2D(lanelet.centerline()),
    to2D(lanelet::utils::conversion::toLaneletPoint(point)).basicPoint());
}
}  // namespace

// RouteHandler 类的构造函数
RouteHandler::RouteHandler(const LaneletMapBin & map_msg)
{
  setMap(map_msg);  // 设置地图
  route_ptr_ = nullptr; // 初始化路径指针
}

// 设置地图
void RouteHandler::setMap(const LaneletMapBin & map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>(); // 创建 Lanelet 地图指针
  lanelet::utils::conversion::fromBinMsg(
    map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_); // 从二进制消息加载地图
  const auto map_major_version_opt =
    lanelet::io_handlers::parseMajorVersion(map_msg.version_map_format);  // 解析地图版本
  if (!map_major_version_opt) {
    RCLCPP_WARN(
      logger_, "setMap() for invalid version map: %s", map_msg.version_map_format.c_str()); // 解析地图版本
  } else if (map_major_version_opt.value() > static_cast<uint64_t>(lanelet::autoware::version)) {
    RCLCPP_WARN(
      logger_, "setMap() for a map(version %s) newer than lanelet2_extension support version(%d)",
      map_msg.version_map_format.c_str(), static_cast<int>(lanelet::autoware::version));
  }

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle); // 创建交通规则
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);  // 创建行人交通规则
  const lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules); // 构建车辆路由图
  const lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);  // 构建行人路由图
  const lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});  // 创建整体路由图
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);  // 共享整体路由图
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);  // 获取所有车道

  is_map_msg_ready_ = true; // 设置地图消息已准备好
  is_handler_ready_ = false;  // 设置处理器未准备好

  setLaneletsFromRouteMsg();  // 从路径消息设置车道
}

// 检查路径是否循环
bool RouteHandler::isRouteLooped(const RouteSections & route_sections)
{
  std::set<lanelet::Id> lane_primitives;  // 创建车道 ID 集合
  for (const auto & route_section : route_sections) {
    for (const auto & primitive : route_section.primitives) {
      if (lane_primitives.find(primitive.id) != lane_primitives.end()) {
        return true;  // 如果找到重复 ID，则路径循环
      }
      lane_primitives.emplace(primitive.id);
    }
  }
  return false; // 如果没有重复 ID，则路径不循环
}

// 设置路径
void RouteHandler::setRoute(const LaneletRoute & route_msg)
{
  if (!isRouteLooped(route_msg.segments)) {
    // 如果路径未修改但为新路径，则重置原始起点
    if (!route_ptr_ || route_ptr_->uuid != route_msg.uuid) {
      original_start_pose_ = route_msg.start_pose;  // 设置原始起点
      original_goal_pose_ = route_msg.goal_pose;  // 设置原始终点
    }
    route_ptr_ = std::make_shared<LaneletRoute>(route_msg); // 创建路径指针
    is_handler_ready_ = false;  // 设置处理器未准备好
    setLaneletsFromRouteMsg();  // 从路径消息设置车道
  } else {
    RCLCPP_ERROR(
      logger_,
      "Loop detected within route! Currently, no loop is allowed for route! Using previous route");
  }
}

// 检查处理器是否准备好
bool RouteHandler::isHandlerReady() const
{
  return is_handler_ready_;
}

// 设置路径车道
void RouteHandler::setRouteLanelets(const lanelet::ConstLanelets & path_lanelets)
{
  if (!path_lanelets.empty()) {
    const auto & first_lanelet = path_lanelets.front();
    start_lanelets_ = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, first_lanelet);  // 获取起点车道
    const auto & last_lanelet = path_lanelets.back();
    goal_lanelets_ = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, last_lanelet);  // 获取终点车道
  }

  // 设置路径车道
  std::unordered_set<lanelet::Id> route_lanelets_id;
  std::unordered_set<lanelet::Id> candidate_lanes_id;
  for (const auto & lane : path_lanelets) {
    route_lanelets_id.insert(lane.id());
    const auto right_relations = routing_graph_ptr_->rightRelations(lane);
    for (const auto & right_relation : right_relations) {
      if (right_relation.relationType == lanelet::routing::RelationType::Right) {
        route_lanelets_id.insert(right_relation.lanelet.id());
      } else if (right_relation.relationType == lanelet::routing::RelationType::AdjacentRight) {
        candidate_lanes_id.insert(right_relation.lanelet.id());
      }
    }
    const auto left_relations = routing_graph_ptr_->leftRelations(lane);
    for (const auto & left_relation : left_relations) {
      if (left_relation.relationType == lanelet::routing::RelationType::Left) {
        route_lanelets_id.insert(left_relation.lanelet.id());
      } else if (left_relation.relationType == lanelet::routing::RelationType::AdjacentLeft) {
        candidate_lanes_id.insert(left_relation.lanelet.id());
      }
    }
  }

  // 检查候选车道是否为路径的一部分
  for (const auto & candidate_id : candidate_lanes_id) {
    lanelet::ConstLanelet lanelet = lanelet_map_ptr_->laneletLayer.get(candidate_id);
    auto previous_lanelets = routing_graph_ptr_->previous(lanelet);
    bool is_connected_to_main_lanes_prev = false;
    bool is_connected_to_candidate_prev = true;
    if (exists(start_lanelets_, lanelet)) {
      is_connected_to_candidate_prev = false;
    }
    while (!previous_lanelets.empty() && is_connected_to_candidate_prev &&
           !is_connected_to_main_lanes_prev) {
      is_connected_to_candidate_prev = false;

      for (const auto & prev_lanelet : previous_lanelets) {
        if (route_lanelets_id.find(prev_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_prev = true;
          break;
        }
        if (exists(start_lanelets_, prev_lanelet)) {
          break;
        }

        if (candidate_lanes_id.find(prev_lanelet.id()) != candidate_lanes_id.end()) {
          is_connected_to_candidate_prev = true;
          previous_lanelets = routing_graph_ptr_->previous(prev_lanelet);
          break;
        }
      }
    }

    auto following_lanelets = routing_graph_ptr_->following(lanelet);
    bool is_connected_to_main_lanes_next = false;
    bool is_connected_to_candidate_next = true;
    if (exists(goal_lanelets_, lanelet)) {
      is_connected_to_candidate_next = false;
    }
    while (!following_lanelets.empty() && is_connected_to_candidate_next &&
           !is_connected_to_main_lanes_next) {
      is_connected_to_candidate_next = false;
      for (const auto & next_lanelet : following_lanelets) {
        if (route_lanelets_id.find(next_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_next = true;
          break;
        }
        if (exists(goal_lanelets_, next_lanelet)) {
          break;
        }
        if (candidate_lanes_id.find(next_lanelet.id()) != candidate_lanes_id.end()) {
          is_connected_to_candidate_next = true;
          following_lanelets = routing_graph_ptr_->following(next_lanelet);
          break;
        }
      }
    }

    if (is_connected_to_main_lanes_next && is_connected_to_main_lanes_prev) {
      route_lanelets_id.insert(candidate_id);
    }
  }

  route_lanelets_.clear();
  route_lanelets_.reserve(route_lanelets_id.size());
  for (const auto & id : route_lanelets_id) {
    route_lanelets_.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  is_handler_ready_ = true;
}

// 清除路径
void RouteHandler::clearRoute()
{
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  start_lanelets_.clear();
  goal_lanelets_.clear();
  route_ptr_ = nullptr;
  is_handler_ready_ = false;
}

// 从路径消息设置车道
void RouteHandler::setLaneletsFromRouteMsg()
{
  if (!route_ptr_ || !is_map_msg_ready_) {
    return;
  }
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  const bool is_route_valid = lanelet::utils::route::isRouteValid(*route_ptr_, lanelet_map_ptr_);
  if (!is_route_valid) {
    return;
  }

  size_t primitive_size{0};
  for (const auto & route_section : route_ptr_->segments) {
    primitive_size += route_section.primitives.size();
  }
  route_lanelets_.reserve(primitive_size);

  for (const auto & route_section : route_ptr_->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      route_lanelets_.push_back(llt);
      if (id == route_section.preferred_primitive.id) {
        preferred_lanelets_.push_back(llt);
      }
    }
  }
  goal_lanelets_.clear();
  start_lanelets_.clear();
  if (!route_ptr_->segments.empty()) {
    goal_lanelets_.reserve(route_ptr_->segments.back().primitives.size());
    for (const auto & primitive : route_ptr_->segments.back().primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      goal_lanelets_.push_back(llt);
    }
    start_lanelets_.reserve(route_ptr_->segments.front().primitives.size());
    for (const auto & primitive : route_ptr_->segments.front().primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      start_lanelets_.push_back(llt);
    }
  }
  is_handler_ready_ = true;
}

// 获取路径头信息
Header RouteHandler::getRouteHeader() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getRouteHeader: Route has not been set yet");
    return Header();
  }
  return route_ptr_->header;
}

// 获取路径 UUID
UUID RouteHandler::getRouteUuid() const
{
  if (!route_ptr_) {
    RCLCPP_WARN_SKIPFIRST(logger_, "[Route Handler] getRouteUuid: Route has not been set yet");
    return UUID();
  }
  return route_ptr_->uuid;
}

// 检查是否允许修改目标点
bool RouteHandler::isAllowedGoalModification() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getRouteUuid: Route has not been set yet");
    return false;
  }
  return route_ptr_->allow_modification;
}

// 获取目标点之后的车道
std::vector<lanelet::ConstLanelet> RouteHandler::getLanesAfterGoal(
  const double vehicle_length) const
{
  lanelet::ConstLanelet goal_lanelet;
  if (!getGoalLanelet(&goal_lanelet)) {
    return std::vector<lanelet::ConstLanelet>{};
  }

  const double min_succeeding_length = vehicle_length * 2;
  const auto succeeding_lanes_vec = lanelet::utils::query::getSucceedingLaneletSequences(
    routing_graph_ptr_, goal_lanelet, min_succeeding_length);
  if (succeeding_lanes_vec.empty()) {
    return std::vector<lanelet::ConstLanelet>{};
  }

  return succeeding_lanes_vec.front();
}

// 获取路径车道
lanelet::ConstLanelets RouteHandler::getRouteLanelets() const
{
  return route_lanelets_;
}

// 获取首选车道
lanelet::ConstLanelets RouteHandler::getPreferredLanelets() const
{
  return preferred_lanelets_;
}

// 获取起点姿态
Pose RouteHandler::getStartPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getStartPose: Route has not been set yet");
    return Pose();
  }
  return route_ptr_->start_pose;
}

// 获取原始起点姿态
Pose RouteHandler::getOriginalStartPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getOriginalStartPose: Route has not been set yet");
    return Pose();
  }
  return original_start_pose_;
}

// 获取终点姿态
Pose RouteHandler::getGoalPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getGoalPose: Route has not been set yet");
    return Pose();
  }
  return route_ptr_->goal_pose;
}

// 获取原始终点姿态
Pose RouteHandler::getOriginalGoalPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getOriginalGoalPose: Route has not been set yet");
    return Pose();
  }
  return original_goal_pose_;
}

// 获取目标车道 ID
lanelet::Id RouteHandler::getGoalLaneId() const
{
  if (!route_ptr_ || route_ptr_->segments.empty()) {
    return lanelet::InvalId;
  }

  return route_ptr_->segments.back().preferred_primitive.id;
}

// 获取目标车道
bool RouteHandler::getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const
{
  const lanelet::Id goal_lane_id = getGoalLaneId();
  for (const auto & llt : route_lanelets_) {
    if (llt.id() == goal_lane_id) {
      *goal_lanelet = llt;
      return true;
    }
  }
  return false;
}

// 检查车道是否在目标路径段中
bool RouteHandler::isInGoalRouteSection(const lanelet::ConstLanelet & lanelet) const
{
  if (!route_ptr_ || route_ptr_->segments.empty()) {
    return false;
  }
  return exists(route_ptr_->segments.back().primitives, lanelet.id());
}

// 根据 ID 获取车道
lanelet::ConstLanelets RouteHandler::getLaneletsFromIds(const lanelet::Ids & ids) const
{
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(ids.size());
  for (const auto & id : ids) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  return lanelets;
}

// 根据 ID 获取单个车道
lanelet::ConstLanelet RouteHandler::getLaneletsFromId(const lanelet::Id id) const
{
  return lanelet_map_ptr_->laneletLayer.get(id);
}

// 检查车道是否为死胡同
bool RouteHandler::isDeadEndLanelet(const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelet next_lanelet;
  return !getNextLaneletWithinRoute(lanelet, &next_lanelet);
}

// 获取车道可变道的相邻车道
lanelet::ConstLanelets RouteHandler::getLaneChangeableNeighbors(
  const lanelet::ConstLanelet & lanelet) const
{
  return lanelet::utils::query::getLaneChangeableNeighbors(routing_graph_ptr_, lanelet);
}

// 获取车道序列（向前）
lanelet::ConstLanelets RouteHandler::getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length, const bool only_route_lanes) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_forward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet)) {
      if (only_route_lanes) {
        break;
      }
      const auto next_lanes = getNextLanelets(current_lanelet);
      if (next_lanes.empty()) {
        break;
      }
      next_lanelet = next_lanes.front();
    }
    // 检查循环
    if (lanelet.id() == next_lanelet.id()) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(next_lanelet.centerline().basicLineString()));
  }

  return lanelet_sequence_forward;
}

// 获取车道序列（向后）
lanelet::ConstLanelets RouteHandler::getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length, const bool only_route_lanes) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  double length = 0;
  lanelet::ConstLanelets previous_lanelets;

  auto checkForLoop =
    [&lanelet](const lanelet::ConstLanelets & lanelets_to_check, const bool is_route_lanelets) {
      if (is_route_lanelets) {
        return std::none_of(
          lanelets_to_check.begin(), lanelets_to_check.end(),
          [lanelet](auto & prev_llt) { return lanelet.id() != prev_llt.id(); });
      }
      return std::any_of(
        lanelets_to_check.begin(), lanelets_to_check.end(),
        [lanelet](auto & prev_llt) { return lanelet.id() == prev_llt.id(); });
    };

  auto isNewLanelet = [&lanelet,
                       &lanelet_sequence_backward](const lanelet::ConstLanelet & lanelet_to_check) {
    if (lanelet.id() == lanelet_to_check.id()) return false;
    return std::none_of(
      lanelet_sequence_backward.begin(), lanelet_sequence_backward.end(),
      [&lanelet_to_check](auto & backward) { return (backward.id() == lanelet_to_check.id()); });
  };

  while (rclcpp::ok() && length < min_length) {
    previous_lanelets.clear();
    bool is_route_lanelets = true;
    if (!getPreviousLaneletsWithinRoute(current_lanelet, &previous_lanelets)) {
      if (only_route_lanes) break;
      previous_lanelets = getPreviousLanelets(current_lanelet);
      if (previous_lanelets.empty()) break;
      is_route_lanelets = false;
    }

    if (checkForLoop(previous_lanelets, is_route_lanelets)) break;

    for (const auto & prev_lanelet : previous_lanelets) {
      if (!isNewLanelet(prev_lanelet) || exists(goal_lanelets_, prev_lanelet)) continue;
      lanelet_sequence_backward.push_back(prev_lanelet);
      length +=
        static_cast<double>(boost::geometry::length(prev_lanelet.centerline().basicLineString()));
      current_lanelet = prev_lanelet;
      break;
    }
  }

  std::reverse(lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  return lanelet_sequence_backward;
}

// 此函数的目的就是为了获取指定车道的完整序列，可以获得当前位置的车道的前后完整序列。
lanelet::ConstLanelets RouteHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const double backward_distance,
  const double forward_distance, const bool only_route_lanes) const
{
  Pose current_pose{};
  current_pose.orientation.w = 1;
  if (!lanelet.centerline().empty()) {
    current_pose.position = lanelet::utils::conversion::toGeomMsgPt(lanelet.centerline().front());
  }

  lanelet::ConstLanelets lanelet_sequence;
  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  // 获取向前的车道序列
  const lanelet::ConstLanelets lanelet_sequence_forward = std::invoke([&]() {
    if (only_route_lanes) {
      return getLaneletSequenceAfter(lanelet, forward_distance, only_route_lanes);
    } else if (isShoulderLanelet(lanelet)) {
      return getShoulderLaneletSequenceAfter(lanelet, forward_distance);
    }
    return lanelet::ConstLanelets{};
  });

  // 获取向后的车道序列
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      if (only_route_lanes) {
        return getLaneletSequenceUpTo(lanelet, backward_distance, only_route_lanes);
      } else if (isShoulderLanelet(lanelet)) {
        return getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
      }
    }
    return lanelet::ConstLanelets{};
  });

  // 检查是否存在循环
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }

  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  lanelet_sequence.push_back(lanelet);
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  // 返回完整的车道序列
  return lanelet_sequence;
}

// 获取车道序列（双向）
lanelet::ConstLanelets RouteHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & current_pose, const double backward_distance,
  const double forward_distance, const bool only_route_lanes) const
{
  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return {};
  }

  lanelet::ConstLanelets lanelet_sequence_forward =
    getLaneletSequenceAfter(lanelet, forward_distance, only_route_lanes);
  lanelet::ConstLanelets lanelet_sequence = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      return getLaneletSequenceUpTo(lanelet, backward_distance, only_route_lanes);
    }
    return lanelet::ConstLanelets{};
  });

  // 检查循环
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence.empty()) {
    if (lanelet_sequence.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.push_back(lanelet);
  std::move(
    lanelet_sequence_forward.begin(), lanelet_sequence_forward.end(),
    std::back_inserter(lanelet_sequence));
  return lanelet_sequence;
}

// 获取姿态处的车道
lanelet::ConstLanelets RouteHandler::getRoadLaneletsAtPose(const Pose & pose) const
{
  lanelet::ConstLanelets road_lanelets_at_pose;
  const lanelet::BasicPoint2d p{pose.position.x, pose.position.y};
  const auto lanelets_at_pose = lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p));
  for (const auto & lanelet_at_pose : lanelets_at_pose) {
    // 确认姿态是否在车道内，因为 "search" 使用的是近似矩形
    const auto is_pose_inside_lanelet = lanelet::geometry::inside(lanelet_at_pose, p);
    if (is_pose_inside_lanelet && isRoadLanelet(lanelet_at_pose))
      road_lanelets_at_pose.push_back(lanelet_at_pose);
  }
  return road_lanelets_at_pose;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getFollowingShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  bool found = false;
  const auto & search_point = lanelet.centerline().back().basicPoint2d();
  const auto next_lanelet = lanelet_map_ptr_->laneletLayer.nearestUntil(
    search_point, [&](const auto & bbox, const auto & ll) {
      if (isShoulderLanelet(ll) && lanelet::geometry::follows(lanelet, ll)) found = true;
      // stop search once next shoulder lanelet is found, or the bbox does not touch the search
      // point
      return found || lanelet::geometry::distance2d(bbox, search_point) > 1e-3;
    });
  if (found && next_lanelet.has_value()) return *next_lanelet;
  return std::nullopt;
}

// 获取左侧肩部车道
std::optional<lanelet::ConstLanelet> RouteHandler::getLeftShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound())) {
    if (other_lanelet.rightBound() == lanelet.leftBound() && isShoulderLanelet(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

// 获取右侧肩部车道
std::optional<lanelet::ConstLanelet> RouteHandler::getRightShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound())) {
    if (other_lanelet.leftBound() == lanelet.rightBound() && isShoulderLanelet(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

// 获取姿态处的肩部车道
lanelet::ConstLanelets RouteHandler::getShoulderLaneletsAtPose(const Pose & pose) const
{
  lanelet::ConstLanelets lanelets_at_pose;
  const lanelet::BasicPoint2d p{pose.position.x, pose.position.y};
  const auto candidates_at_pose = lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p));
  for (const auto & candidate : candidates_at_pose) {
    // confirm that the pose is inside the lanelet since "search" does an approximation with boxes
    const auto is_pose_inside_lanelet = lanelet::geometry::inside(candidate, p);
    if (is_pose_inside_lanelet && isShoulderLanelet(candidate))
      lanelets_at_pose.push_back(candidate);
  }
  return lanelets_at_pose;
}

// 获取肩部车道序列（向前）
lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!isShoulderLanelet(lanelet)) return lanelet_sequence_forward;

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::set<lanelet::Id> searched_ids{};
  while (rclcpp::ok() && length < min_length) {
    const auto next_lanelet = getFollowingShoulderLanelet(current_lanelet);
    if (!next_lanelet) break;
    lanelet_sequence_forward.push_back(*next_lanelet);
    if (searched_ids.find(next_lanelet->id()) != searched_ids.end()) {
      // loop shoulder detected
      break;
    }
    searched_ids.insert(next_lanelet->id());
    current_lanelet = *next_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(next_lanelet->centerline().basicLineString()));
  }

  return lanelet_sequence_forward;
}

// 获取前一个肩部车道
std::optional<lanelet::ConstLanelet> RouteHandler::getPreviousShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  bool found = false;
  const auto & search_point = lanelet.centerline().front().basicPoint2d();
  const auto previous_lanelet = lanelet_map_ptr_->laneletLayer.nearestUntil(
    search_point, [&](const auto & bbox, const auto & ll) {
      if (isShoulderLanelet(ll) && lanelet::geometry::follows(ll, lanelet)) found = true;
      // 如果找到前一个肩部车道，或者矩形不接触搜索点，则停止搜索
      return found || lanelet::geometry::distance2d(bbox, search_point) > 1e-3;
    });
  if (found && previous_lanelet.has_value()) return *previous_lanelet;
  return std::nullopt;
}

// 获取肩部车道序列（向后）
lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!isShoulderLanelet(lanelet)) return lanelet_sequence_backward;

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::set<lanelet::Id> searched_ids{};
  while (rclcpp::ok() && length < min_length) {
    const auto prev_lanelet = getPreviousShoulderLanelet(current_lanelet);
    if (!prev_lanelet) break;

    lanelet_sequence_backward.insert(lanelet_sequence_backward.begin(), *prev_lanelet);
    if (searched_ids.find(prev_lanelet->id()) != searched_ids.end()) {
      // loop shoulder detected
      break;
    }
    searched_ids.insert(prev_lanelet->id());
    current_lanelet = *prev_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(prev_lanelet->centerline().basicLineString()));
  }

  return lanelet_sequence_backward;
}

// 获取肩部车道序列（双向）
lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & pose, const double backward_distance,
  const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  if (!isShoulderLanelet(lanelet)) {
    return lanelet_sequence;
  }

  lanelet::ConstLanelets lanelet_sequence_forward =
    getShoulderLaneletSequenceAfter(lanelet, forward_distance);
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, pose);
    if (arc_coordinate.length < backward_distance) {
      return getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
    }
    return lanelet::ConstLanelets{};
  });

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());

  lanelet_sequence.push_back(lanelet);
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

// 在路径内获取最近的车道
bool RouteHandler::getClosestLaneletWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
{
  return lanelet::utils::query::getClosestLanelet(route_lanelets_, search_pose, closest_lanelet);
}

// 在路径内获取最近的首选车道
bool RouteHandler::getClosestPreferredLaneletWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
{
  return lanelet::utils::query::getClosestLanelet(
    preferred_lanelets_, search_pose, closest_lanelet);
}

// 在路径内获取最近的车道（带约束）
bool RouteHandler::getClosestLaneletWithConstrainsWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet, const double dist_threshold,
  const double yaw_threshold) const
{
  return lanelet::utils::query::getClosestLaneletWithConstrains(
    route_lanelets_, search_pose, closest_lanelet, dist_threshold, yaw_threshold);
}

// 从车道获取最近的路径车道
bool RouteHandler::getClosestRouteLaneletFromLanelet(
  const Pose & search_pose, const lanelet::ConstLanelet & reference_lanelet,
  lanelet::ConstLanelet * closest_lanelet, const double dist_threshold,
  const double yaw_threshold) const
{
  lanelet::ConstLanelets previous_lanelets, next_lanelets, lanelet_sequence;
  if (getPreviousLaneletsWithinRoute(reference_lanelet, &previous_lanelets)) {
    lanelet_sequence = previous_lanelets;
  }

  lanelet_sequence.push_back(reference_lanelet);

  if (getNextLaneletsWithinRoute(reference_lanelet, &next_lanelets)) {
    lanelet_sequence.insert(lanelet_sequence.end(), next_lanelets.begin(), next_lanelets.end());
  }

  if (lanelet::utils::query::getClosestLaneletWithConstrains(
        lanelet_sequence, search_pose, closest_lanelet, dist_threshold, yaw_threshold)) {
    return true;
  }

  return false;
}

// 在路径内获取下一个车道
bool RouteHandler::getNextLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * next_lanelets) const
{
  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }

  const auto start_lane_id = route_ptr_->segments.front().preferred_primitive.id;

  const auto following_lanelets = routing_graph_ptr_->following(lanelet);
  next_lanelets->clear();
  for (const auto & llt : following_lanelets) {
    if (start_lane_id != llt.id() && exists(route_lanelets_, llt)) {
      next_lanelets->push_back(llt);
    }
  }
  return !(next_lanelets->empty());
}

// 在路径内获取下一个车道
bool RouteHandler::getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const
{
  lanelet::ConstLanelets next_lanelets{};
  if (getNextLaneletsWithinRoute(lanelet, &next_lanelets)) {
    *next_lanelet = next_lanelets.front();
    return true;
  }
  return false;
}

// 获取下一个车道
lanelet::ConstLanelets RouteHandler::getNextLanelets(const lanelet::ConstLanelet & lanelet) const
{
  return routing_graph_ptr_->following(lanelet);
}

// 在路径内获取前一个车道
bool RouteHandler::getPreviousLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * prev_lanelets) const
{
  if (exists(start_lanelets_, lanelet)) {
    return false;
  }
  const auto candidate_lanelets = routing_graph_ptr_->previous(lanelet);
  prev_lanelets->clear();
  for (const auto & llt : candidate_lanelets) {
    if (exists(route_lanelets_, llt)) {
      prev_lanelets->push_back(llt);
    }
  }
  return !(prev_lanelets->empty());
}

// 获取前一个车道
lanelet::ConstLanelets RouteHandler::getPreviousLanelets(
  const lanelet::ConstLanelet & lanelet) const
{
  return routing_graph_ptr_->previous(lanelet);
}

// 获取右侧车道
std::optional<lanelet::ConstLanelet> RouteHandler::getRightLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // 获取右侧道路车道（肩部车道）
  if (isShoulderLanelet(lanelet)) {
    const auto right_lanelets = lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound());
    for (const auto & right_lanelet : right_lanelets)
      if (isRoadLanelet(right_lanelet)) return right_lanelet;
    return std::nullopt;
  }

  // 获取右侧肩部车道
  if (get_shoulder_lane) {
    const auto right_shoulder_lanelet = getRightShoulderLanelet(lanelet);
    if (right_shoulder_lanelet) return *right_shoulder_lanelet;
  }

  // 可路由车道
  const auto & right_lane = routing_graph_ptr_->right(lanelet);
  if (right_lane) {
    return *right_lane;
  }

  // 非可路由车道（例如，车道变更不可行）
  const auto & adjacent_right_lane = routing_graph_ptr_->adjacentRight(lanelet);
  if (adjacent_right_lane) {
    return *adjacent_right_lane;
  }

  // 相同根的右侧车道
  if (!enable_same_root) {
    return std::nullopt;
  }

  lanelet::ConstLanelets prev_lanelet;
  if (!getPreviousLaneletsWithinRoute(lanelet, &prev_lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelet next_lanelet;
  if (!getNextLaneletWithinRoute(lanelet, &next_lanelet)) {
    for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
      if (lanelet.rightBound().back().id() == lane.leftBound().back().id()) {
        return lane;
      }
    }
    return std::nullopt;
  }

  const auto next_right_lane = getRightLanelet(next_lanelet, false);
  if (!next_right_lane) {
    return std::nullopt;
  }

  for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
    for (const auto & target_lane : getNextLanelets(lane)) {
      if (next_right_lane.value().id() == target_lane.id()) {
        return lane;
      }
    }
  }

  return std::nullopt;
}

// 获取左侧车道
std::optional<lanelet::ConstLanelet> RouteHandler::getLeftLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // left road lanelet of shoulder lanelet
  if (isShoulderLanelet(lanelet)) {
    const auto left_lanelets = lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound());
    for (const auto & left_lanelet : left_lanelets)
      if (isRoadLanelet(left_lanelet)) return left_lanelet;
    return std::nullopt;
  }

  // left shoulder lanelet
  if (get_shoulder_lane) {
    const auto left_shoulder_lanelet = getLeftShoulderLanelet(lanelet);
    if (left_shoulder_lanelet) return *left_shoulder_lanelet;
  }

  // routable lane
  const auto & left_lane = routing_graph_ptr_->left(lanelet);
  if (left_lane) {
    return *left_lane;
  }

  // non-routable lane (e.g. lane change infeasible)
  const auto & adjacent_left_lane = routing_graph_ptr_->adjacentLeft(lanelet);
  if (adjacent_left_lane) {
    return *adjacent_left_lane;
  }

  // same root right lanelet
  if (!enable_same_root) {
    return std::nullopt;
  }

  lanelet::ConstLanelets prev_lanelet;
  if (!getPreviousLaneletsWithinRoute(lanelet, &prev_lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelet next_lanelet;
  if (!getNextLaneletWithinRoute(lanelet, &next_lanelet)) {
    for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
      if (lanelet.leftBound().back().id() == lane.rightBound().back().id()) {
        return lane;
      }
    }
    return std::nullopt;
  }

  const auto next_left_lane = getLeftLanelet(next_lanelet, false);
  if (!next_left_lane) {
    return std::nullopt;
  }

  for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
    for (const auto & target_lane : getNextLanelets(lane)) {
      if (next_left_lane.value().id() == target_lane.id()) {
        return lane;
      }
    }
  }

  return std::nullopt;
}

// 获取右侧对向车道
lanelet::Lanelets RouteHandler::getRightOppositeLanelets(
  const lanelet::ConstLanelet & lanelet) const
{
  const auto opposite_candidate_lanelets =
    lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound().invert());

  lanelet::Lanelets opposite_lanelets;
  for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
    if (candidate_lanelet.leftBound().id() == lanelet.rightBound().id()) {
      continue;
    }

    opposite_lanelets.push_back(candidate_lanelet);
  }

  return opposite_lanelets;
}

// 获取所有左侧共享线字符串车道
lanelet::ConstLanelets RouteHandler::getAllLeftSharedLinestringLanelets(
  const lanelet::ConstLanelet & lane, const bool & include_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets linestring_shared;
  try {
    auto lanelet_at_left = getLeftLanelet(lane);
    auto lanelet_at_left_opposite = getLeftOppositeLanelets(lane);
    while (lanelet_at_left) {
      linestring_shared.push_back(lanelet_at_left.value());
      lanelet_at_left = getLeftLanelet(lanelet_at_left.value());
      if (!lanelet_at_left) {
        break;
      }
      lanelet_at_left_opposite = getLeftOppositeLanelets(lanelet_at_left.value());
    }

    if (!lanelet_at_left_opposite.empty() && include_opposite) {
      if (invert_opposite) {
        linestring_shared.push_back(lanelet_at_left_opposite.front().invert());
      } else {
        linestring_shared.push_back(lanelet_at_left_opposite.front());
      }
      auto lanelet_at_right = getRightLanelet(lanelet_at_left_opposite.front());
      while (lanelet_at_right) {
        if (invert_opposite) {
          linestring_shared.push_back(lanelet_at_right.value().invert());
        } else {
          linestring_shared.push_back(lanelet_at_right.value());
        }
        lanelet_at_right = getRightLanelet(lanelet_at_right.value());
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "Exception in getAllLeftSharedLinestringLanelets: " << e.what() << std::endl;
    return {};
  } catch (...) {
    std::cerr << "Unknown exception in getAllLeftSharedLinestringLanelets" << std::endl;
    return {};
  }
  return linestring_shared;
}

// 获取所有右侧共享线字符串车道
lanelet::ConstLanelets RouteHandler::getAllRightSharedLinestringLanelets(
  const lanelet::ConstLanelet & lane, const bool & include_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets linestring_shared;
  try {
    auto lanelet_at_right = getRightLanelet(lane);
    auto lanelet_at_right_opposite = getRightOppositeLanelets(lane);
    while (lanelet_at_right) {
      linestring_shared.push_back(lanelet_at_right.value());
      lanelet_at_right = getRightLanelet(lanelet_at_right.value());
      if (!lanelet_at_right) {
        break;
      }
      lanelet_at_right_opposite = getRightOppositeLanelets(lanelet_at_right.value());
    }

    if (!lanelet_at_right_opposite.empty() && include_opposite) {
      if (invert_opposite) {
        linestring_shared.push_back(lanelet_at_right_opposite.front().invert());
      } else {
        linestring_shared.push_back(lanelet_at_right_opposite.front());
      }
      auto lanelet_at_left = getLeftLanelet(lanelet_at_right_opposite.front());
      while (lanelet_at_left) {
        if (invert_opposite) {
          linestring_shared.push_back(lanelet_at_left.value().invert());
        } else {
          linestring_shared.push_back(lanelet_at_left.value());
        }
        lanelet_at_left = getLeftLanelet(lanelet_at_left.value());
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "Exception in getAllRightSharedLinestringLanelets: " << e.what() << std::endl;
    return {};
  } catch (...) {
    std::cerr << "Unknown exception in getAllRightSharedLinestringLanelets" << std::endl;
    return {};
  }
  return linestring_shared;
}

// 获取所有共享线字符串车道
lanelet::ConstLanelets RouteHandler::getAllSharedLineStringLanelets(
  const lanelet::ConstLanelet & current_lane, bool is_right, bool is_left, bool is_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets shared{current_lane};

  if (is_right) {
    const lanelet::ConstLanelets all_right_lanelets =
      getAllRightSharedLinestringLanelets(current_lane, is_opposite, invert_opposite);
    shared.insert(shared.end(), all_right_lanelets.begin(), all_right_lanelets.end());
  }

  if (is_left) {
    const lanelet::ConstLanelets all_left_lanelets =
      getAllLeftSharedLinestringLanelets(current_lane, is_opposite, invert_opposite);
    shared.insert(shared.end(), all_left_lanelets.begin(), all_left_lanelets.end());
  }

  return shared;
}

// 获取左侧对向车道
lanelet::Lanelets RouteHandler::getLeftOppositeLanelets(const lanelet::ConstLanelet & lanelet) const
{
  const auto opposite_candidate_lanelets =
    lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound().invert());

  lanelet::Lanelets opposite_lanelets;
  for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
    if (candidate_lanelet.rightBound().id() == lanelet.leftBound().id()) {
      continue;
    }

    opposite_lanelets.push_back(candidate_lanelet);
  }

  return opposite_lanelets;
}

// 获取最右侧车道
lanelet::ConstLanelet RouteHandler::getMostRightLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // recursively compute the width of the lanes
  const auto & same = getRightLanelet(lanelet, enable_same_root, get_shoulder_lane);

  if (same) {
    return getMostRightLanelet(same.value(), enable_same_root, get_shoulder_lane);
  }

  return lanelet;
}

// 获取最左侧车道
lanelet::ConstLanelet RouteHandler::getMostLeftLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // 递归计算车道宽度
  const auto & same = getLeftLanelet(lanelet, enable_same_root, get_shoulder_lane);

  if (same) {
    return getMostLeftLanelet(same.value(), enable_same_root, get_shoulder_lane);
  }

  return lanelet;
}

// 获取前一个车道序列
std::vector<lanelet::ConstLanelets> RouteHandler::getPrecedingLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const double length,
  const lanelet::ConstLanelets & exclude_lanelets) const
{
  return lanelet::utils::query::getPrecedingLaneletSequences(
    routing_graph_ptr_, lanelet, length, exclude_lanelets);
}

// 获取车道变更目标
std::optional<lanelet::ConstLanelet> RouteHandler::getLaneChangeTarget(
  const lanelet::ConstLanelets & lanelets, const Direction direction) const
{
  for (const auto & lanelet : lanelets) {
    const int num = getNumLaneToPreferredLane(lanelet, direction);
    if (num == 0) {
      continue;
    }

    if (direction == Direction::NONE || direction == Direction::RIGHT) {
      if (num < 0) {
        const auto right_lanes = routing_graph_ptr_->right(lanelet);
        if (!!right_lanes) {
          return *right_lanes;
        }
      }
    }

    if (direction == Direction::NONE || direction == Direction::LEFT) {
      if (num > 0) {
        const auto left_lanes = routing_graph_ptr_->left(lanelet);
        if (!!left_lanes) {
          return *left_lanes;
        }
      }
    }
  }

  return std::nullopt;
}

// 获取除首选车道外的车道变更目标
std::optional<lanelet::ConstLanelet> RouteHandler::getLaneChangeTargetExceptPreferredLane(
  const lanelet::ConstLanelets & lanelets, const Direction direction) const
{
  for (const auto & lanelet : lanelets) {
    if (direction == Direction::RIGHT) {
      // 如果首选车道在左侧，则获取右侧车道
      if (getNumLaneToPreferredLane(lanelet, direction) < 0) {
        continue;
      }

      const auto right_lanes = routing_graph_ptr_->right(lanelet);
      if (!!right_lanes) {
        return *right_lanes;
      }
    }

    if (direction == Direction::LEFT) {
      // 如果首选车道在右侧，则获取左侧车道
      if (getNumLaneToPreferredLane(lanelet, direction) > 0) {
        continue;
      }
      const auto left_lanes = routing_graph_ptr_->left(lanelet);
      if (!!left_lanes) {
        return *left_lanes;
      }
    }
  }

  return std::nullopt;
}

// 获取靠边停车目标车道
std::optional<lanelet::ConstLanelet> RouteHandler::getPullOverTarget(const Pose & goal_pose) const
{
  const lanelet::BasicPoint2d p(goal_pose.position.x, goal_pose.position.y);
  constexpr auto search_distance = 0.1;
  const lanelet::BasicPoint2d offset(search_distance, search_distance);
  const auto lanelets_in_range =
    lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p - offset, p + offset));
  for (const auto & lanelet : lanelets_in_range) {
    const auto is_in_lanelet = lanelet::utils::isInLanelet(goal_pose, lanelet, search_distance);
    if (is_in_lanelet && isShoulderLanelet(lanelet)) return lanelet;
  }
  return std::nullopt;
}

// 获取靠边停车起始车道
std::optional<lanelet::ConstLanelet> RouteHandler::getPullOutStartLane(
  const Pose & pose, const double vehicle_width) const
{
  const lanelet::BasicPoint2d p(pose.position.x, pose.position.y);
  const auto search_distance = vehicle_width / 2.0;
  const lanelet::BasicPoint2d offset(search_distance, search_distance);
  const auto lanelets_in_range =
    lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p - offset, p + offset));
  for (const auto & lanelet : lanelets_in_range) {
    const auto is_in_lanelet = lanelet::utils::isInLanelet(pose, lanelet, search_distance);
    if (is_in_lanelet && isShoulderLanelet(lanelet)) return lanelet;
  }
  return std::nullopt;
}

// 获取到首选车道的车道数
int RouteHandler::getNumLaneToPreferredLane(
  const lanelet::ConstLanelet & lanelet, const Direction direction) const
{
  if (exists(preferred_lanelets_, lanelet)) {
    return 0;
  }

  if ((direction == Direction::NONE) || (direction == Direction::RIGHT)) {
    int num{0};
    const auto & right_lanes =
      lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet);
    for (const auto & right : right_lanes) {
      num--;
      if (exists(preferred_lanelets_, right)) {
        return num;
      }
    }
  }

  if ((direction == Direction::NONE) || (direction == Direction::LEFT)) {
    const auto & left_lanes =
      lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet);
    int num = 0;
    for (const auto & left : left_lanes) {
      num++;
      if (exists(preferred_lanelets_, left)) {
        return num;
      }
    }
  }

  return 0;  // TODO(Horibe) check if return 0 is appropriate.
}

// 获取到首选车道的横向间隔
std::vector<double> RouteHandler::getLateralIntervalsToPreferredLane(
  const lanelet::ConstLanelet & lanelet, const Direction direction) const
{
  if (exists(preferred_lanelets_, lanelet)) {
    return {};
  }

  if ((direction == Direction::NONE) || (direction == Direction::RIGHT)) {
    std::vector<double> intervals;
    lanelet::ConstLanelet current_lanelet = lanelet;
    const auto & right_lanes =
      lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet);
    for (const auto & right : right_lanes) {
      const auto & current_centerline = current_lanelet.centerline();
      const auto & next_centerline = right.centerline();
      if (current_centerline.empty() || next_centerline.empty()) {
        return intervals;
      }
      const auto & curr_pt = current_centerline.front();
      const auto & next_pt = next_centerline.front();
      intervals.push_back(-lanelet::geometry::distance2d(to2D(curr_pt), to2D(next_pt)));

      if (exists(preferred_lanelets_, right)) {
        return intervals;
      }
      current_lanelet = right;
    }
  }

  if ((direction == Direction::NONE) || (direction == Direction::LEFT)) {
    std::vector<double> intervals;
    lanelet::ConstLanelet current_lanelet = lanelet;
    const auto & left_lanes =
      lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet);
    for (const auto & left : left_lanes) {
      const auto & current_centerline = current_lanelet.centerline();
      const auto & next_centerline = left.centerline();
      if (current_centerline.empty() || next_centerline.empty()) {
        return intervals;
      }
      const auto & curr_pt = current_centerline.front();
      const auto & next_pt = next_centerline.front();
      intervals.push_back(lanelet::geometry::distance2d(to2D(curr_pt), to2D(next_pt)));

      if (exists(preferred_lanelets_, left)) {
        return intervals;
      }
      current_lanelet = left;
    }
  }

  return {};
}

// 根据车道序列生成中心线路径
PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
  bool use_exact) const
{
  using lanelet::utils::to2D;
  using lanelet::utils::conversion::toLaneletPoint;

  // 1. 根据车道中心线计算参考点
  // 注意：此向量与车道序列对齐
  std::vector<PiecewiseReferencePoints> piecewise_ref_points_vec;
  const auto add_ref_point = [&](const auto & pt) {
    piecewise_ref_points_vec.back().push_back(
      ReferencePoint{false, lanelet::utils::conversion::toGeomMsgPt(pt)});
  };
  double s = 0;
  for (const auto & llt : lanelet_sequence) {
    piecewise_ref_points_vec.push_back(std::vector<ReferencePoint>{});

    const lanelet::ConstLineString3d centerline = llt.centerline();
    for (size_t i = 0; i < centerline.size(); i++) {
      const auto & pt = centerline[i];
      const lanelet::ConstPoint3d next_pt =
        (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];
      const double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

      if (s < s_start && s + distance > s_start) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_start) : pt;
        add_ref_point(p);
      }
      if (s >= s_start && s <= s_end) {
        add_ref_point(pt);
      }
      if (s < s_end && s + distance > s_end) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_end) : next_pt;
        add_ref_point(p);
      }
      s += distance;
    }
  }

  // 2. 计算路径点
  const auto waypoints_vec = calcWaypointsVector(lanelet_sequence);

  // 3. 去除路径点的边缘点
  for (const auto & waypoints : waypoints_vec) {
    for (auto piecewise_waypoints_itr = waypoints.begin();
         piecewise_waypoints_itr != waypoints.end(); ++piecewise_waypoints_itr) {
      const auto & piecewise_waypoints = piecewise_waypoints_itr->piecewise_waypoints;
      const auto lanelet_id = piecewise_waypoints_itr->lanelet_id;

      // 计算与 piecewise_waypoints 对应的车道序列索引
      const auto lanelet_sequence_itr = std::find_if(
        lanelet_sequence.begin(), lanelet_sequence.end(),
        [&](const auto & lanelet) { return lanelet.id() == lanelet_id; });
      if (lanelet_sequence_itr == lanelet_sequence.end()) {
        continue;
      }
      const size_t piecewise_waypoints_lanelet_sequence_index =
        std::distance(lanelet_sequence.begin(), lanelet_sequence_itr);

      // 根据路径点计算参考点
      const auto ref_points_by_waypoints = convertWaypointsToReferencePoints(piecewise_waypoints);

      // 根据路径点更新参考点
      const bool is_first_waypoint_contained = piecewise_waypoints_itr == waypoints.begin();
      const bool is_last_waypoint_contained = piecewise_waypoints_itr == waypoints.end() - 1;
      if (is_first_waypoint_contained || is_last_waypoint_contained) {
        // 如果 piecewise_waypoints_itr 是路径点的起始或结束

        const auto original_piecewise_ref_points =
          piecewise_ref_points_vec.at(piecewise_waypoints_lanelet_sequence_index);

        // 定义 current_piecewise_ref_points，并用路径点初始化
        auto & current_piecewise_ref_points =
          piecewise_ref_points_vec.at(piecewise_waypoints_lanelet_sequence_index);
        current_piecewise_ref_points = ref_points_by_waypoints;
        if (is_first_waypoint_contained) {
          // 将原始参考点添加到当前参考点，并去除与路径点重叠的参考点
          current_piecewise_ref_points.insert(
            current_piecewise_ref_points.begin(), original_piecewise_ref_points.begin(),
            original_piecewise_ref_points.end());
          const bool is_removing_direction_forward = false;
          removeOverlappedCenterlineWithWaypoints(
            piecewise_ref_points_vec, piecewise_waypoints, lanelet_sequence,
            piecewise_waypoints_lanelet_sequence_index, is_removing_direction_forward);
        }
        if (is_last_waypoint_contained) {
          // 将原始参考点添加到当前参考点，并去除与路径点重叠的参考点
          current_piecewise_ref_points.insert(
            current_piecewise_ref_points.end(), original_piecewise_ref_points.begin(),
            original_piecewise_ref_points.end());
          const bool is_removing_direction_forward = true;
          removeOverlappedCenterlineWithWaypoints(
            piecewise_ref_points_vec, piecewise_waypoints, lanelet_sequence,
            piecewise_waypoints_lanelet_sequence_index, is_removing_direction_forward);
        }
      } else {
        // 如果 piecewise_waypoints_itr 不是路径点的起始或结束，
        // 则移除所有参考点并添加路径点。
        piecewise_ref_points_vec.at(piecewise_waypoints_lanelet_sequence_index) =
          ref_points_by_waypoints;
      }
    }
  }

  // 4. 转换为 PathPointsWithLaneIds
  PathWithLaneId reference_path{};
  for (size_t lanelet_idx = 0; lanelet_idx < lanelet_sequence.size(); ++lanelet_idx) {
    const auto & lanelet = lanelet_sequence.at(lanelet_idx);
    const float speed_limit =
      static_cast<float>(traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value());

    const auto & piecewise_ref_points = piecewise_ref_points_vec.at(lanelet_idx);
    for (const auto & ref_point : piecewise_ref_points) {
      PathPointWithLaneId p{};
      p.point.pose.position = ref_point.point;
      p.lane_ids.push_back(lanelet.id());
      p.point.longitudinal_velocity_mps = speed_limit;
      reference_path.points.push_back(p);
    }
  }
  reference_path = removeOverlappingPoints(reference_path);

  // 如果只有一个点，则添加一个点以便计算偏航角
  if (reference_path.points.size() == 1) {
    const lanelet::Id lane_id = reference_path.points.front().lane_ids.front();
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
    const auto point = reference_path.points.front().point.pose.position;
    const auto lane_yaw = lanelet::utils::getLaneletAngle(lanelet, point);
    PathPointWithLaneId path_point{};
    path_point.lane_ids.push_back(lane_id);
    constexpr double ds{0.1};
    path_point.point.pose.position.x = point.x + ds * std::cos(lane_yaw);
    path_point.point.pose.position.y = point.y + ds * std::sin(lane_yaw);
    path_point.point.pose.position.z = point.z;
    reference_path.points.push_back(path_point);
  }

  // 设置角度
  for (size_t i = 0; i < reference_path.points.size(); i++) {
    double angle{0.0};
    const auto & pts = reference_path.points;
    if (i + 1 < reference_path.points.size()) {
      angle = autoware_utils::calc_azimuth_angle(
        pts.at(i).point.pose.position, pts.at(i + 1).point.pose.position);
    } else if (i != 0) {
      angle = autoware_utils::calc_azimuth_angle(
        pts.at(i - 1).point.pose.position, pts.at(i).point.pose.position);
    }
    reference_path.points.at(i).point.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(angle);
  }

  return reference_path;
}

// 计算路径点向量
std::vector<Waypoints> RouteHandler::calcWaypointsVector(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  std::vector<Waypoints> waypoints_vec;
  for (size_t lanelet_idx = 0; lanelet_idx < lanelet_sequence.size(); ++lanelet_idx) {
    const auto & lanelet = lanelet_sequence.at(lanelet_idx);
    if (!lanelet.hasAttribute("waypoints")) {
      continue;
    }

    // 生成分段路径点
    PiecewiseWaypoints piecewise_waypoints{lanelet.id(), {}};
    const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
    for (const auto & waypoint : lanelet_map_ptr_->lineStringLayer.get(waypoints_id)) {
      piecewise_waypoints.piecewise_waypoints.push_back(
        lanelet::utils::conversion::toGeomMsgPt(waypoint));
    }
    if (piecewise_waypoints.piecewise_waypoints.empty()) {
      continue;
    }

    // 检查分段路径点是否与前一个分段路径点相连
    if (
      !waypoints_vec.empty() && isClose(
                                  waypoints_vec.back().back().piecewise_waypoints.back(),
                                  piecewise_waypoints.piecewise_waypoints.front(), 1.0)) {
      waypoints_vec.back().push_back(piecewise_waypoints);
    } else {
      // 添加新的路径点
      Waypoints new_waypoints;
      new_waypoints.push_back(piecewise_waypoints);
      waypoints_vec.push_back(new_waypoints);
    }
  }

  return waypoints_vec;
}

// 去除与路径点重叠的中心线
void RouteHandler::removeOverlappedCenterlineWithWaypoints(
  std::vector<PiecewiseReferencePoints> & piecewise_ref_points_vec,
  const std::vector<geometry_msgs::msg::Point> & piecewise_waypoints,
  const lanelet::ConstLanelets & lanelet_sequence,
  const size_t piecewise_waypoints_lanelet_sequence_index,
  const bool is_removing_direction_forward) const
{
  const double waypoints_interpolation_arc_margin_ratio = 10.0;

  // 计算弧长阈值
  const double front_arc_length_threshold = [&]() {
    const auto front_waypoint_arc_coordinates = calcArcCoordinates(
      lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index), piecewise_waypoints.front());
    const double lanelet_arc_length = boost::geometry::length(
      lanelet::utils::to2D(lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index)
                             .centerline()
                             .basicLineString()));
    return -lanelet_arc_length + front_waypoint_arc_coordinates.length -
           std::abs(front_waypoint_arc_coordinates.distance) *
             waypoints_interpolation_arc_margin_ratio;
  }();
  const double back_arc_length_threshold = [&]() {
    const auto back_waypoint_arc_coordinates = calcArcCoordinates(
      lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index), piecewise_waypoints.back());
    return back_waypoint_arc_coordinates.length + std::abs(back_waypoint_arc_coordinates.distance) *
                                                    waypoints_interpolation_arc_margin_ratio;
  }();

  double offset_arc_length = 0.0;
  int target_lanelet_sequence_index = static_cast<int>(piecewise_waypoints_lanelet_sequence_index);
  while (isIndexWithinVector(lanelet_sequence, target_lanelet_sequence_index)) {
    auto & target_piecewise_ref_points = piecewise_ref_points_vec.at(target_lanelet_sequence_index);
    const double target_lanelet_arc_length = boost::geometry::length(lanelet::utils::to2D(
      lanelet_sequence.at(target_lanelet_sequence_index).centerline().basicLineString()));

    // 在目标车道中搜索重叠的参考点
    std::vector<size_t> overlapped_ref_points_indices{};
    const bool is_search_finished = [&]() {
      for (size_t ref_point_unsigned_index = 0;
           ref_point_unsigned_index < target_piecewise_ref_points.size();
           ++ref_point_unsigned_index) {
        const size_t ref_point_index =
          is_removing_direction_forward
            ? ref_point_unsigned_index
            : target_piecewise_ref_points.size() - 1 - ref_point_unsigned_index;
        const auto & ref_point = target_piecewise_ref_points.at(ref_point_index);

        // skip waypoints
        if (ref_point.is_waypoint) {
          if (
            target_lanelet_sequence_index ==
            static_cast<int>(piecewise_waypoints_lanelet_sequence_index)) {
            overlapped_ref_points_indices.clear();
          }
          continue;
        }

        const double ref_point_arc_length =
          (is_removing_direction_forward ? 0 : -target_lanelet_arc_length) +
          calcArcCoordinates(lanelet_sequence.at(target_lanelet_sequence_index), ref_point.point)
            .length;
        if (is_removing_direction_forward) {
          if (back_arc_length_threshold < offset_arc_length + ref_point_arc_length) {
            return true;
          }
        } else {
          if (offset_arc_length + ref_point_arc_length < front_arc_length_threshold) {
            return true;
          }
        }

        overlapped_ref_points_indices.push_back(ref_point_index);
      }
      return false;
    }();

    // 在目标车道中搜索重叠的参考点
    removeIndicesFromVector(target_piecewise_ref_points, overlapped_ref_points_indices);

    // 如果搜索重叠中心线完成，则退出
    if (is_search_finished) {
      break;
    }

    target_lanelet_sequence_index += is_removing_direction_forward ? 1 : -1;
    offset_arc_length = (is_removing_direction_forward ? 1 : -1) * target_lanelet_arc_length;
  }
}

// 检查地图消息是否准备好
bool RouteHandler::isMapMsgReady() const
{
  return is_map_msg_ready_;
}

// 获取路由图指针
lanelet::routing::RoutingGraphPtr RouteHandler::getRoutingGraphPtr() const
{
  return routing_graph_ptr_;
}

// 获取交通规则指针
lanelet::traffic_rules::TrafficRulesPtr RouteHandler::getTrafficRulesPtr() const
{
  return traffic_rules_ptr_;
}

// 获取整体路由图指针
std::shared_ptr<const lanelet::routing::RoutingGraphContainer> RouteHandler::getOverallGraphPtr()
  const
{
  return overall_graphs_ptr_;
}

// 获取 Lanelet 地图指针
lanelet::LaneletMapPtr RouteHandler::getLaneletMapPtr() const
{
  return lanelet_map_ptr_;
}

// 检查车道是否为肩部车道
bool RouteHandler::isShoulderLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
         lanelet.attribute(lanelet::AttributeName::Subtype) == "road_shoulder";
}

// 检查车道是否为路径车道
bool RouteHandler::isRouteLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet::utils::contains(route_lanelets_, lanelet);
}

// 检查车道是否为道路车道
bool RouteHandler::isRoadLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
         lanelet.attribute(lanelet::AttributeName::Subtype) == lanelet::AttributeValueString::Road;
}

// 获取前一个车道序列
lanelet::ConstLanelets RouteHandler::getPreviousLaneletSequence(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  // 定义一个容器来存储前一个车道序列
  lanelet::ConstLanelets previous_lanelet_sequence;
  if (lanelet_sequence.empty()) {
    return previous_lanelet_sequence;
  }

  // 如果输入的车道序列为空，则直接返回空序列
  const auto & first_lane = lanelet_sequence.front();

  // 如果第一个车道已经是起点车道，则无法继续向前追溯，直接返回空序列
  if (exists(start_lanelets_, first_lane)) {
    return previous_lanelet_sequence;
  }

  // 获取第一个车道右侧的所有相邻车道
  auto right_relations =
    lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, first_lane);
  
  // 遍历右侧相邻车道，尝试找到前一个车道序列
  for (const auto & right : right_relations) {
    previous_lanelet_sequence = getLaneletSequenceUpTo(right);  // 获取该相邻车道的完整序列
    if (!previous_lanelet_sequence.empty()) { // 如果找到非空序列，则返回
      return previous_lanelet_sequence;
    }
  }

  // 获取第一个车道左侧的所有相邻车道
  auto left_relations = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, first_lane);

  // 遍历左侧相邻车道，尝试找到前一个车道序列
  for (const auto & left : left_relations) {
    previous_lanelet_sequence = getLaneletSequenceUpTo(left); // 获取该相邻车道的完整序列
    if (!previous_lanelet_sequence.empty()) { // 如果找到非空序列，则返回
      return previous_lanelet_sequence;
    }
  }

  // 如果没有找到合适的前一个车道序列，则返回空序列
  return previous_lanelet_sequence;
}

// 获取路径内的相邻车道
lanelet::ConstLanelets RouteHandler::getNeighborsWithinRoute(
  const lanelet::ConstLanelet & lanelet) const
{
  // 获取指定车道的所有相邻车道
  // 但这个函数实际上也只是做了一个容器转换，真正获取的是调用了lanelet::utils::query::getAllNeighbors函数。
  // 这个函数也是是lanelet2库中的工具函数，用于查询车道的邻居关系。
  const lanelet::ConstLanelets neighbor_lanelets =
    lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, lanelet);

  // 定义一个容器来存储属于当前规划路径的相邻车道
  lanelet::ConstLanelets neighbors_within_route;

  // 遍历所有相邻车道
  for (const auto & llt : neighbor_lanelets) {
    // 检查当前相邻车道是否属于当前规划路径
    if (exists(route_lanelets_, llt)) {
      // 如果属于，则将其添加到结果中
      neighbors_within_route.push_back(llt);
    }
  }

  // 返回筛选后的相邻车道
  return neighbors_within_route;
}

// 根据起始点和目标点，规划路径，所规划出来的路径便是path_lanelets，它是从起点到终点，连续的路径段。
bool RouteHandler::planPathLaneletsBetweenCheckpoints(
  const Pose & start_checkpoint, const Pose & goal_checkpoint,
  lanelet::ConstLanelets * path_lanelets, const bool consider_no_drivable_lanes) const
{
  // Find lanelets for start point. First, find all lanelets containing the start point to calculate
  // all possible route later. It fails when the point is not located on any road lanelet (e.g. the
  // start point is located out of any lanelets or road_shoulder lanelet which is not contained in
  // road lanelet). In that case, find the closest lanelet instead (within some maximum range).

  // 定义最大搜索范围，用于查找起点和终点附近的车道
  constexpr auto max_search_range = 20.0;

  // 尝试获取起点处的道路车道
  auto start_lanelets = getRoadLaneletsAtPose(start_checkpoint);

  // 如果起点不在任何道路车道上，则尝试在最大搜索范围内查找最近的车道
  lanelet::ConstLanelet start_lanelet;
  if (start_lanelets.empty()) {
    const lanelet::BasicPoint2d p(start_checkpoint.position.x, start_checkpoint.position.y);
    const lanelet::BoundingBox2d bbox(
      lanelet::BasicPoint2d(p.x() - max_search_range, p.y() - max_search_range),
      lanelet::BasicPoint2d(p.x() + max_search_range, p.y() + max_search_range));
    // std::as_const(*ptr) to use the const version of the search function
    auto candidates = std::as_const(*lanelet_map_ptr_).laneletLayer.search(bbox);
    candidates.erase(
      std::remove_if(
        candidates.begin(), candidates.end(), [&](const auto & l) { return !isRoadLanelet(l); }),
      candidates.end());
    if (lanelet::utils::query::getClosestLanelet(candidates, start_checkpoint, &start_lanelet))
      start_lanelets = {start_lanelet};
  }

  // 如果仍然找不到起点车道，则返回错误
  if (start_lanelets.empty()) {
    RCLCPP_WARN_STREAM(
      logger_, "Failed to find current lanelet."
                 << std::endl
                 << " - start checkpoint: " << toString(start_checkpoint) << std::endl
                 << " - goal checkpoint: " << toString(goal_checkpoint) << std::endl);
    return false;
  }

  // 尝试获取终点处的道路车道
  lanelet::ConstLanelet goal_lanelet;
  const lanelet::BasicPoint2d p(goal_checkpoint.position.x, goal_checkpoint.position.y);
  const lanelet::BoundingBox2d bbox(
    lanelet::BasicPoint2d(p.x() - max_search_range, p.y() - max_search_range),
    lanelet::BasicPoint2d(p.x() + max_search_range, p.y() + max_search_range));
  auto candidates = std::as_const(*lanelet_map_ptr_).laneletLayer.search(bbox);
  candidates.erase(
    std::remove_if(
      candidates.begin(), candidates.end(), [&](const auto & l) { return !isRoadLanelet(l); }),
    candidates.end());
  // if there is a lanelet in candidates that is included in previous preferred lanelets,
  // set it as goal_lanelet.
  // this is to select the same lane as much as possible when rerouting with waypoints.
  const auto findGoalClosestPreferredLanelet = [&]() -> std::optional<lanelet::ConstLanelet> {
    lanelet::ConstLanelet closest_lanelet;
    if (getClosestPreferredLaneletWithinRoute(goal_checkpoint, &closest_lanelet)) {
      if (std::find(candidates.begin(), candidates.end(), closest_lanelet) != candidates.end()) {
        if (lanelet::utils::isInLanelet(goal_checkpoint, closest_lanelet)) {
          return closest_lanelet;
        }
      }
    }
    if (getClosestLaneletWithinRoute(goal_checkpoint, &closest_lanelet)) {
      if (std::find(candidates.begin(), candidates.end(), closest_lanelet) != candidates.end()) {
        if (lanelet::utils::isInLanelet(goal_checkpoint, closest_lanelet)) {
          std::stringstream preferred_lanelets_str;
          for (const auto & preferred_lanelet : preferred_lanelets_) {
            preferred_lanelets_str << preferred_lanelet.id() << ", ";
          }
          RCLCPP_WARN(
            logger_,
            "Failed to find reroute on previous preferred lanelets %s, but on previous route "
            "segment %ld still",
            preferred_lanelets_str.str().c_str(), closest_lanelet.id());
          return closest_lanelet;
        }
      }
    }
    return std::nullopt;
  };
  if (auto closest_lanelet = findGoalClosestPreferredLanelet()) {
    goal_lanelet = closest_lanelet.value();
  } else {
    if (!lanelet::utils::query::getClosestLanelet(candidates, goal_checkpoint, &goal_lanelet)) {
      RCLCPP_WARN_STREAM(
        logger_, "Failed to find closest lanelet."
                   << std::endl
                   << " - start checkpoint: " << toString(start_checkpoint) << std::endl
                   << " - goal checkpoint: " << toString(goal_checkpoint) << std::endl);
      return false;
    }
  }

  // 定义变量以存储可能的路径和最短路径
  lanelet::Optional<lanelet::routing::Route> optional_route;
  lanelet::routing::LaneletPath shortest_path;
  bool is_route_found = false;

  // 定义角度差异的最大值和阈值
  double smallest_angle_diff = std::numeric_limits<double>::max();
  constexpr double yaw_threshold = M_PI / 2.0;

  for (const auto & st_llt : start_lanelets) {
    // check if the angle difference between start_checkpoint and start lanelet center line
    // orientation is in yaw_threshold range
    // 计算起点姿态与车道中心线方向之间的角度差异
    double lanelet_angle = lanelet::utils::getLaneletAngle(st_llt, start_checkpoint.position);
    double pose_yaw = tf2::getYaw(start_checkpoint.orientation);
    double angle_diff = std::abs(autoware_utils::normalize_radian(lanelet_angle - pose_yaw));

    // 检查角度差异是否在允许范围内
    bool is_proper_angle = angle_diff <= std::abs(yaw_threshold);

    // 使用路由图计算从起点车道到终点车道的路径
    // 这个的是是在lanelet2库中实现的，这是一个外部依赖库。它位于lanelet2_routing包中。
    // 这个函数是RoutingGraph类的成员函数，用于计算两个lanelet之间的路径。
    optional_route = routing_graph_ptr_->getRoute(st_llt, goal_lanelet, 0);
    if (!optional_route || !is_proper_angle) {
      RCLCPP_ERROR_STREAM(
        logger_, "Failed to find a proper route!"
                   << std::endl
                   << " - start checkpoint: " << toString(start_checkpoint) << std::endl
                   << " - goal checkpoint: " << toString(goal_checkpoint) << std::endl
                   << " - start lane id: " << st_llt.id() << std::endl
                   << " - goal lane id: " << goal_lanelet.id() << std::endl);
      continue;
    }

    // 标记找到了合适的路径
    is_route_found = true;

    // 如果找到了首选车道且与当前起点车道匹配，则直接选择该路径
    lanelet::ConstLanelet preferred_lane{};
    if (getClosestPreferredLaneletWithinRoute(start_checkpoint, &preferred_lane)) {
      if (st_llt.id() == preferred_lane.id()) {
        shortest_path = optional_route->shortestPath();
        start_lanelet = st_llt;
        break;
      }
    }

    // 如果当前角度差异更小，则更新最短路径和起点车道
    if (angle_diff < smallest_angle_diff) {
      smallest_angle_diff = angle_diff;
      shortest_path = optional_route->shortestPath();
      start_lanelet = st_llt;
    }
  }

  // 如果找到了合适的路径，则进一步处理路径
  if (is_route_found) {
    lanelet::routing::LaneletPath path;
    // 如果需要考虑不可行驶车道，则尝试查找可行驶的路径
    path = [&]() -> lanelet::routing::LaneletPath {
      if (consider_no_drivable_lanes && hasNoDrivableLaneInPath(shortest_path)) {
        const auto drivable_lane_path = findDrivableLanePath(start_lanelet, goal_lanelet);
        if (drivable_lane_path) return *drivable_lane_path;
      }
      return shortest_path;
    }();

    // 将路径车道序列存储到输出参数中
    path_lanelets->reserve(path.size());
    for (const auto & llt : path) {
      path_lanelets->push_back(llt);
    }
  }

  // 返回是否成功找到路径
  return is_route_found;
}

// 根据给定的车道序列（path_lanelets）创建地图段（LaneletSegment）
std::vector<LaneletSegment> RouteHandler::createMapSegments(
  const lanelet::ConstLanelets & path_lanelets) const
{
  const auto main_path = getMainLanelets(path_lanelets);  // 获取主车道序列
  std::vector<LaneletSegment> route_sections; // 用于存储最终的地图段

  if (main_path.empty()) {  // 如果主车道序列为空，直接返回空结果
    return route_sections;
  }

  route_sections.reserve(main_path.size()); // 预分配内存以提高效率
  for (const auto & main_llt : main_path) { // 遍历每个主车道
    LaneletSegment route_section_msg; // 获取当前主车道的相邻车道
    const lanelet::ConstLanelets route_section_lanelets = getNeighborsWithinRoute(main_llt);
    route_section_msg.preferred_primitive.id = main_llt.id(); // 设置首选车道ID
    route_section_msg.primitives.reserve(route_section_lanelets.size());  // 预分配内存
    for (const auto & section_llt : route_section_lanelets) { // 遍历相邻车道
      LaneletPrimitive p; // 创建一个车道原语对象
      p.id = section_llt.id();  // 设置车道ID
      p.primitive_type = "lane";  // 设置车道类型为 "lane"
      route_section_msg.primitives.push_back(p);  // 将车道原语添加到地图段中
    }
    route_sections.push_back(route_section_msg);   // 将当前地图段添加到结果列表中
  }
  return route_sections;  // 返回所有地图段
}

// 获取主车道
lanelet::ConstLanelets RouteHandler::getMainLanelets(
  const lanelet::ConstLanelets & path_lanelets) const
{
  // 从路径的最后一个车道开始获取车道序列
  auto lanelet_sequence = getLaneletSequence(path_lanelets.back());

  // 打印调试信息
  RCLCPP_INFO_STREAM(logger_, "getMainLanelets: lanelet_sequence = " << lanelet_sequence);

  lanelet::ConstLanelets main_lanelets;
  while (!lanelet_sequence.empty()) {
    // 将当前车道序列插入到主车道序列的开头
    main_lanelets.insert(main_lanelets.begin(), lanelet_sequence.begin(), lanelet_sequence.end());
    // 获取前一个车道序列
    lanelet_sequence = getPreviousLaneletSequence(lanelet_sequence);
  }
  return main_lanelets; // 返回主车道序列
}

// 检查车道是否为不可行驶车道
bool RouteHandler::isNoDrivableLane(const lanelet::ConstLanelet & llt)
{
  const std::string no_drivable_lane_attribute = llt.attributeOr("no_drivable_lane", "no");
  return no_drivable_lane_attribute == "yes";
}

// 检查路径中是否有不可行驶车道
bool RouteHandler::hasNoDrivableLaneInPath(const lanelet::routing::LaneletPath & path) const
{
  for (const auto & llt : path)
    if (isNoDrivableLane(llt)) return true;
  return false;
}

// 查找可行驶车道路径
std::optional<lanelet::routing::LaneletPath> RouteHandler::findDrivableLanePath(
  const lanelet::ConstLanelet & start_lanelet, const lanelet::ConstLanelet & goal_lanelet) const
{
  // 我们创建一个新的路由图，其中不可行驶车道的成本为无穷大
  const auto drivable_routing_graph_ptr = lanelet::routing::RoutingGraph::build(
    *lanelet_map_ptr_, *traffic_rules_ptr_,
    lanelet::routing::RoutingCostPtrs{std::make_shared<RoutingCostDrivable>()});
  const auto route = drivable_routing_graph_ptr->getRoute(start_lanelet, goal_lanelet, 0);
  if (route) return route->shortestPath();
  return {};
}

// 根据弧长获取姿态
Pose RouteHandler::get_pose_from_2d_arc_length(
  const lanelet::ConstLanelets & lanelet_sequence, const double s) const
{
  double accumulated_distance2d = 0;
  for (const auto & llt : lanelet_sequence) {
    const auto & centerline = llt.centerline();
    for (auto it = centerline.begin(); std::next(it) != centerline.end(); ++it) {
      const auto pt = *it;
      const auto next_pt = *std::next(it);
      const double distance2d = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));
      if (accumulated_distance2d + distance2d > s) {
        const double ratio = (s - accumulated_distance2d) / distance2d;
        const auto interpolated_pt = pt.basicPoint() * (1 - ratio) + next_pt.basicPoint() * ratio;
        const auto yaw = std::atan2(next_pt.y() - pt.y(), next_pt.x() - pt.x());
        Pose pose;
        pose.position = create_point(interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z());
        pose.orientation = create_quaternion_from_yaw(yaw);
        return pose;
      }
      accumulated_distance2d += distance2d;
    }
  }
  return Pose{};
}
}  // namespace autoware::route_handler
