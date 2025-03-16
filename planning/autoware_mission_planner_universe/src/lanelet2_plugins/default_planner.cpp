// Copyright 2019-2024 Autoware Foundation
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

#include "default_planner.hpp"

#include "utility_functions.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp> 
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/is_empty.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <tf2/utils.h>

#include <limits>
#include <vector>

namespace autoware::mission_planner_universe::lanelet2
{

// 初始化默认路径规划器的通用部分
void DefaultPlanner::RR(rclcpp::Node * node)
{
  is_graph_ready_ = false; // 标记图未准备好
  node_ = node;

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  // 创建目标点足迹的调试标记发布器
  pub_goal_footprint_marker_ =
    node_->create_publisher<MarkerArray>("~/debug/goal_footprint", durable_qos);

  // 获取车辆信息
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo();
  // 声明参数
  param_.goal_angle_threshold_deg = node_->declare_parameter<double>("goal_angle_threshold_deg");
  param_.enable_correct_goal_pose = node_->declare_parameter<bool>("enable_correct_goal_pose");
  param_.consider_no_drivable_lanes = node_->declare_parameter<bool>("consider_no_drivable_lanes");
  param_.check_footprint_inside_lanes =
    node_->declare_parameter<bool>("check_footprint_inside_lanes");
}

// 初始化默认路径规划器（通过订阅地图）
void DefaultPlanner::initialize(rclcpp::Node * node)
{
  initialize_common(node);
  // 创建地图订阅器
  map_subscriber_ = node_->create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{10}.transient_local(),
    std::bind(&DefaultPlanner::map_callback, this, std::placeholders::_1));
}

// 初始化默认路径规划器（通过传入地图消息）
void DefaultPlanner::initialize(rclcpp::Node * node, const LaneletMapBin::ConstSharedPtr msg)
{
  initialize_common(node);
  map_callback(msg); // 直接处理地图消息
}

// 检查路径规划器是否准备好
bool DefaultPlanner::ready() const
{
  return is_graph_ready_;
}

// 地图回调函数
void DefaultPlanner::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  route_handler_.setMap(*msg);
  is_graph_ready_ = true;
}

// 可视化路径
PlannerPlugin::MarkerArray DefaultPlanner::visualize(const LaneletRoute & route) const
{
  lanelet::ConstLanelets route_lanelets; // 路径车道
  lanelet::ConstLanelets end_lanelets;  // 结束车道
  lanelet::ConstLanelets goal_lanelets; // 目标车道

  // 遍历路径段，提取车道信息
  for (const auto & route_section : route.segments) {
    for (const auto & lane_id : route_section.primitives) {
      auto lanelet = route_handler_.getLaneletsFromId(lane_id.id);
      route_lanelets.push_back(lanelet);
      if (route_section.preferred_primitive.id == lane_id.id) {
        goal_lanelets.push_back(lanelet); // 目标车道
      } else {
        end_lanelets.push_back(lanelet);  // 结束车道
      }
    }
  }

  // 定义颜色
  const std_msgs::msg::ColorRGBA cl_route =
    autoware_utils::create_marker_color(0.8, 0.99, 0.8, 0.15);
  const std_msgs::msg::ColorRGBA cl_ll_borders =
    autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.999);
  const std_msgs::msg::ColorRGBA cl_end = autoware_utils::create_marker_color(0.2, 0.2, 0.4, 0.05);
  const std_msgs::msg::ColorRGBA cl_goal = autoware_utils::create_marker_color(0.2, 0.4, 0.4, 0.05);

  // 创建路径标记数组
  visualization_msgs::msg::MarkerArray route_marker_array;
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(route_lanelets, cl_ll_borders, false));
  insert_marker_array(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "route_lanelets", route_lanelets, cl_route));
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("end_lanelets", end_lanelets, cl_end));
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("goal_lanelets", goal_lanelets, cl_goal));

  return route_marker_array;
}

// 可视化调试足迹
visualization_msgs::msg::MarkerArray DefaultPlanner::visualize_debug_footprint(
  autoware_utils::LinearRing2d goal_footprint)
{
  visualization_msgs::msg::MarkerArray msg;
  // 创建默认标记
  auto marker = autoware_utils::create_default_marker(
    "map", rclcpp::Clock().now(), "goal_footprint", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.05, 0.0, 0.0),
    autoware_utils::create_marker_color(0.99, 0.99, 0.2, 1.0));
  marker.lifetime = rclcpp::Duration::from_seconds(2.5);

  // 添加足迹的点
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[0][0], goal_footprint[0][1], 0.0));
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[1][0], goal_footprint[1][1], 0.0));
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[2][0], goal_footprint[2][1], 0.0));
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[3][0], goal_footprint[3][1], 0.0));
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[4][0], goal_footprint[4][1], 0.0));
  marker.points.push_back(
    autoware_utils::create_point(goal_footprint[5][0], goal_footprint[5][1], 0.0));
  marker.points.push_back(marker.points.front());

  msg.markers.push_back(marker);

  return msg;
}


// 获取从起始车道到指定距离内的所有后续车道
lanelet::ConstLanelets next_lanelets_up_to(
  const lanelet::ConstLanelet & start_lanelet, const double up_to_distance,
  const route_handler::RouteHandler & route_handler)
{
  lanelet::ConstLanelets lanelets;
  if (up_to_distance <= 0.0) {
    return lanelets;
  }
  // 获取当前车道的后续车道
  for (const auto & next_lane : route_handler.getNextLanelets(start_lanelet)) {
    lanelets.push_back(next_lane);
    // 递归获取后续车道的后续车道
    const auto next_lanelets = next_lanelets_up_to(
      next_lane, up_to_distance - lanelet::geometry::length2d(next_lane), route_handler);
    lanelets.insert(lanelets.end(), next_lanelets.begin(), next_lanelets.end());
  }
  return lanelets;
}

// 检查目标足迹是否在车道内
bool DefaultPlanner::check_goal_footprint_inside_lanes(
  const lanelet::ConstLanelet & closest_lanelet_to_goal,
  const lanelet::ConstLanelets & path_lanelets,
  const autoware_utils::Polygon2d & goal_footprint) const
{
  autoware_utils::MultiPolygon2d ego_lanes;
  autoware_utils::Polygon2d poly;
  // 遍历路径车道，提取车道的多边形
  for (const auto & ll : path_lanelets) {
    const auto left_shoulder = route_handler_.getLeftShoulderLanelet(ll);
    if (left_shoulder) {
      boost::geometry::convert(left_shoulder->polygon2d().basicPolygon(), poly);
      boost::geometry::correct(poly);
      ego_lanes.push_back(poly);
    }
    const auto right_shoulder = route_handler_.getRightShoulderLanelet(ll);
    if (right_shoulder) {
      boost::geometry::convert(right_shoulder->polygon2d().basicPolygon(), poly);
      boost::geometry::correct(poly);
      ego_lanes.push_back(poly);
    }
    boost::geometry::convert(ll.polygon2d().basicPolygon(), poly);
    boost::geometry::correct(poly);
    ego_lanes.push_back(poly);
  }
  // 获取当前车道的后续车道
  const auto next_lanelets = next_lanelets_up_to(
    closest_lanelet_to_goal, vehicle_info_.max_longitudinal_offset_m, route_handler_);
  for (const auto & ll : next_lanelets) {
    boost::geometry::convert(ll.polygon2d().basicPolygon(), poly);
    boost::geometry::correct(poly);
    ego_lanes.push_back(poly);
  }
  // If the goal is on the very beginning of the closest_lanelet_to_goal, baselink ~ rear part of
  // ego footprint is outside of it. To tolerate it, add previous lanelet
  for (const auto & ll : route_handler_.getPreviousLanelets(closest_lanelet_to_goal)) {
    boost::geometry::convert(ll.polygon2d().basicPolygon(), poly);
    boost::geometry::correct(poly);
    ego_lanes.push_back(poly);
  }

  // 检查目标足迹是否在车道内
  // check if goal footprint is in the ego lane
  autoware_utils::MultiPolygon2d difference;
  boost::geometry::difference(goal_footprint, ego_lanes, difference);
  return boost::geometry::is_empty(difference);
}

// 检查目标点是否有效
bool DefaultPlanner::is_goal_valid(
  const geometry_msgs::msg::Pose & goal, const lanelet::ConstLanelets & path_lanelets)
{
  const auto logger = node_->get_logger();

  const auto goal_lanelet_pt = lanelet::utils::conversion::toLaneletPoint(goal.position);

  // 检查目标点是否在肩道内
  // check if goal is in shoulder lanelet
  lanelet::Lanelet closest_shoulder_lanelet;
  const auto shoulder_lanelets = route_handler_.getShoulderLaneletsAtPose(goal);
  if (lanelet::utils::query::getClosestLanelet(
        shoulder_lanelets, goal, &closest_shoulder_lanelet)) {
    const auto lane_yaw = lanelet::utils::getLaneletAngle(closest_shoulder_lanelet, goal.position);
    const auto goal_yaw = tf2::getYaw(goal.orientation);
    const auto angle_diff = autoware_utils::normalize_radian(lane_yaw - goal_yaw);
    const double th_angle = autoware_utils::deg2rad(param_.goal_angle_threshold_deg);
    if (std::abs(angle_diff) < th_angle) {  // 目标点在肩道内且角度符合要求
      return true;
    }
  }
  lanelet::ConstLanelet closest_lanelet_to_goal;
  const auto road_lanelets_at_goal = route_handler_.getRoadLaneletsAtPose(goal);
  if (!lanelet::utils::query::getClosestLanelet(
        road_lanelets_at_goal, goal, &closest_lanelet_to_goal)) {
    // 如果目标点不在任何车道内，则查找最近的车道
    // if no road lanelets directly at the goal, find the closest one
    const lanelet::BasicPoint2d goal_point{goal.position.x, goal.position.y};
    auto closest_dist = std::numeric_limits<double>::max();
    const auto closest_road_lanelet_found =
      route_handler_.getLaneletMapPtr()->laneletLayer.nearestUntil(
        goal_point, [&](const auto & bbox, const auto & ll) {
          // 通过增加距离搜索最近的车道
          // this search is done by increasing distance between the bounding box and the goal
          // we stop the search when the bounding box is further than the closest dist found
          if (lanelet::geometry::distance2d(bbox, goal_point) > closest_dist)
            return true;  // stop the search  // 停止搜索
          const auto dist = lanelet::geometry::distance2d(goal_point, ll.polygon2d());
          if (route_handler_.isRoadLanelet(ll) && dist < closest_dist) {
            closest_dist = dist;
            closest_lanelet_to_goal = ll;
          }
          return false;  // continue the search // 继续搜索
        });
    if (!closest_road_lanelet_found) return false;  // 如果未找到车道，则目标点无效
  }

  // 获取车辆的足迹
  const auto local_vehicle_footprint = vehicle_info_.createFootprint();
  autoware_utils::LinearRing2d goal_footprint =
    autoware_utils::transform_vector(local_vehicle_footprint, autoware_utils::pose2transform(goal));
  pub_goal_footprint_marker_->publish(visualize_debug_footprint(goal_footprint));
  const auto polygon_footprint = convert_linear_ring_to_polygon(goal_footprint);

  // 检查目标足迹是否超出车道（如果目标点不在停车场内）
  // check if goal footprint exceeds lane when the goal isn't in parking_lot
  if (
    param_.check_footprint_inside_lanes &&
    !check_goal_footprint_inside_lanes(closest_lanelet_to_goal, path_lanelets, polygon_footprint) &&
    !is_in_parking_lot(
      lanelet::utils::query::getAllParkingLots(route_handler_.getLaneletMapPtr()),
      lanelet::utils::conversion::toLaneletPoint(goal.position))) {
    RCLCPP_WARN(logger, "Goal's footprint exceeds lane!");
    return false; // 目标足迹超出车道
  }

  // 检查目标点是否在车道内
  if (is_in_lane(closest_lanelet_to_goal, goal_lanelet_pt)) {
    const auto lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet_to_goal, goal.position);
    const auto goal_yaw = tf2::getYaw(goal.orientation);
    const auto angle_diff = autoware_utils::normalize_radian(lane_yaw - goal_yaw);

    const double th_angle = autoware_utils::deg2rad(param_.goal_angle_threshold_deg);
    if (std::abs(angle_diff) < th_angle) {
      return true;  // 目标点在车道内且角度符合要求
    }
  }

  // 检查目标点是否在停车位内
  // check if goal is in parking space
  const auto parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(route_handler_.getLaneletMapPtr());
  if (is_in_parking_space(parking_spaces, goal_lanelet_pt)) {
    return true;  // 目标点在停车位内
  }

  // 检查目标点是否在停车场内
  // check if goal is in parking lot
  const auto parking_lots =
    lanelet::utils::query::getAllParkingLots(route_handler_.getLaneletMapPtr());
  return is_in_parking_lot(parking_lots, goal_lanelet_pt);  // 目标点在停车场内
}

// 规划路径
PlannerPlugin::LaneletRoute DefaultPlanner::plan(const RoutePoints & points)
{
  const auto logger = node_->get_logger();  // 获取日志记录器

  // 打印路径规划的点
  std::stringstream log_ss;
  for (const auto & point : points) {
    log_ss << "x: " << point.position.x << " " << "y: " << point.position.y << std::endl;
  }
  RCLCPP_DEBUG_STREAM(
    logger, "start planning route with check points: " << std::endl
                                                       << log_ss.str());

  LaneletRoute route_msg; // 定义返回的路径消息
  RouteSections route_sections; // 定义路径段集合

  lanelet::ConstLanelets all_route_lanelets;
  for (std::size_t i = 1; i < points.size(); i++) {
    const auto start_check_point = points.at(i - 1);
    const auto goal_check_point = points.at(i);
    lanelet::ConstLanelets path_lanelets;
    // 规划两个检查点之间的路径车道
    if (!route_handler_.planPathLaneletsBetweenCheckpoints(
          start_check_point, goal_check_point, &path_lanelets, param_.consider_no_drivable_lanes)) {
      RCLCPP_WARN(logger, "Failed to plan route.");
      return route_msg; // 如果规划失败，则返回空路径
    }
    // 将规划的车道添加到路径中
    for (const auto & lane : path_lanelets) {
      if (!all_route_lanelets.empty() && lane.id() == all_route_lanelets.back().id()) continue;
      all_route_lanelets.push_back(lane);
    }
  }
  route_handler_.setRouteLanelets(all_route_lanelets);  // 设置路径车道
  route_sections = route_handler_.createMapSegments(all_route_lanelets);  // 创建路径段

  auto goal_pose = points.back();
  if (param_.enable_correct_goal_pose) {
    // 如果启用了目标点位姿校正，则获取最接近中心线的位姿
    goal_pose = get_closest_centerline_pose(
      lanelet::utils::query::laneletLayer(route_handler_.getLaneletMapPtr()), goal_pose,
      vehicle_info_);
  }

  // 检查目标点是否有效
  if (!is_goal_valid(goal_pose, all_route_lanelets)) {
    RCLCPP_WARN(logger, "Goal is not valid! Please check position and angle of goal_pose");
    return route_msg;
  }

  // 检查路径是否循环
  if (route_handler::RouteHandler::isRouteLooped(route_sections)) {
    RCLCPP_WARN(logger, "Loop detected within route!");
    return route_msg;
  }

  // 校正目标点的高度
  const auto refined_goal = refine_goal_height(goal_pose, route_sections);
  RCLCPP_DEBUG(logger, "Goal Pose Z : %lf", refined_goal.position.z);

  // 设置路径消息
  // The header is assigned by mission planner.
  route_msg.start_pose = points.front();  // 起点位姿
  route_msg.goal_pose = refined_goal;     // 目标点位姿
  route_msg.segments = route_sections;    // 路径段
  return route_msg;
}

// 校正目标点的高度
geometry_msgs::msg::Pose DefaultPlanner::refine_goal_height(
  const Pose & goal, const RouteSections & route_sections)
{
  const auto goal_lane_id = route_sections.back().preferred_primitive.id;
  const auto goal_lanelet = route_handler_.getLaneletsFromId(goal_lane_id);
  const auto goal_lanelet_pt = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const auto goal_height = project_goal_to_map(goal_lanelet, goal_lanelet_pt);

  Pose refined_goal = goal;
  refined_goal.position.z = goal_height;  // 设置校正后的高度
  return refined_goal;
}

// 更新路径
void DefaultPlanner::updateRoute(const PlannerPlugin::LaneletRoute & route)
{
  route_handler_.setRoute(route); // 设置路径处理器中的路径
}

// 清除路径
void DefaultPlanner::clearRoute()
{
  route_handler_.clearRoute();  // 清除路径处理器中的路径
}

}  // namespace autoware::mission_planner_universe::lanelet2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::mission_planner_universe::lanelet2::DefaultPlanner,
  autoware::mission_planner_universe::PlannerPlugin)
