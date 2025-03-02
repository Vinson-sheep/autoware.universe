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

#include "utility_functions.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry.hpp> // 引入 Boost Geometry 库，用于几何计算

#include <lanelet2_core/geometry/Lanelet.h>

#include <limits>
#include <vector>

namespace autoware::mission_planner_universe::lanelet2
{
// 将线性环（LinearRing）转换为多边形（Polygon）
autoware_utils::Polygon2d convert_linear_ring_to_polygon(autoware_utils::LinearRing2d footprint)
{
  autoware_utils::Polygon2d footprint_polygon; // 创建一个多边形对象
  // 将线性环的顶点依次添加到多边形的外环中
  boost::geometry::append(footprint_polygon.outer(), footprint[0]);
  boost::geometry::append(footprint_polygon.outer(), footprint[1]);
  boost::geometry::append(footprint_polygon.outer(), footprint[2]);
  boost::geometry::append(footprint_polygon.outer(), footprint[3]);
  boost::geometry::append(footprint_polygon.outer(), footprint[4]);
  boost::geometry::append(footprint_polygon.outer(), footprint[5]);
  boost::geometry::correct(footprint_polygon); // 修正多边形的几何结构
  return footprint_polygon; // 返回转换后的多边形
}

// 将一个 MarkerArray 插入到另一个 MarkerArray 中
void insert_marker_array(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2)
{
  // 将 a2 的 markers 插入到 a1 的 markers 末尾
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

// 将 Lanelet 的中心线转换为点集
std::vector<geometry_msgs::msg::Point> convertCenterlineToPoints(const lanelet::Lanelet & lanelet)
{
  std::vector<geometry_msgs::msg::Point> centerline_points;
  // 遍历 Lanelet 的中心线，将每个点转换为 geometry_msgs::msg::Point
  for (const auto & point : lanelet.centerline()) {
    geometry_msgs::msg::Point center_point;
    center_point.x = point.basicPoint().x();
    center_point.y = point.basicPoint().y();
    center_point.z = point.basicPoint().z();
    centerline_points.push_back(center_point);
  }
  return centerline_points;
}

// 将 Lanelet 的 BasicPoint3d 转换为 Pose，并设置方向
geometry_msgs::msg::Pose convertBasicPoint3dToPose(
  const lanelet::BasicPoint3d & point, const double lane_yaw)
{
  // calculate new orientation of goal
  geometry_msgs::msg::Pose pose;
  pose.position.x = point.x();
  pose.position.y = point.y();
  pose.position.z = point.z();

  pose.orientation = autoware_utils::create_quaternion_from_yaw(lane_yaw);

  return pose;
}

// 检查点是否在车道内
bool is_in_lane(const lanelet::ConstLanelet & lanelet, const lanelet::ConstPoint3d & point)
{
  // 计算点与车道多边形的最小距离
  // check if goal is on a lane at appropriate angle
  const auto distance = boost::geometry::distance(
    lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(point).basicPoint());
  constexpr double th_distance = std::numeric_limits<double>::epsilon();
  return distance < th_distance; // 如果距离小于阈值，则点在车道内
}

// 检查点是否在停车位内
bool is_in_parking_space(
  const lanelet::ConstLineStrings3d & parking_spaces, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_space : parking_spaces) {
    lanelet::ConstPolygon3d parking_space_polygon;
    // 将停车位线串转换为多边形
    if (!lanelet::utils::lineStringWithWidthToPolygon(parking_space, &parking_space_polygon)) {
      continue;
    }

    // 计算点与停车位多边形的最小距离
    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_space_polygon).basicPolygon(),
      lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) { // 如果距离小于阈值，则点在停车位内
      return true;
    }
  }
  return false;
}

// 检查点是否在停车场内
bool is_in_parking_lot(
  const lanelet::ConstPolygons3d & parking_lots, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_lot : parking_lots) {
    // 计算点与停车场多边形的最小距离
    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_lot).basicPolygon(), lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

// 将目标点投影到地图上，并返回其高度
double project_goal_to_map(
  const lanelet::ConstLanelet & lanelet_component, const lanelet::ConstPoint3d & goal_point)
{
  // 生成车道的精细中心线
  const lanelet::ConstLineString3d center_line =
    lanelet::utils::generateFineCenterline(lanelet_component);
  // 将目标点投影到中心线上
  lanelet::BasicPoint3d project = lanelet::geometry::project(center_line, goal_point.basicPoint());
  return project.z(); // 返回投影点的高度
}

// 获取最接近中心线的位姿
geometry_msgs::msg::Pose get_closest_centerline_pose(
  const lanelet::ConstLanelets & road_lanelets, const geometry_msgs::msg::Pose & point,
  autoware::vehicle_info_utils::VehicleInfo vehicle_info)
{
  lanelet::Lanelet closest_lanelet;
  // 获取最接近的车道
  if (!lanelet::utils::query::getClosestLaneletWithConstrains(
        road_lanelets, point, &closest_lanelet, 0.0)) {
    // 如果点不在任何车道内，则返回原始点
    // point is not on any lanelet.
    return point;
  }

  // 生成车道的精细中心线
  const auto refined_center_line = lanelet::utils::generateFineCenterline(closest_lanelet, 1.0);
  closest_lanelet.setCenterline(refined_center_line);

  // 计算车道的偏航角
  const double lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet, point.position);

  // 找到最接近的点
  const auto nearest_idx = autoware::motion_utils::findNearestIndex(
    convertCenterlineToPoints(closest_lanelet), point.position);
  const auto nearest_point = closest_lanelet.centerline()[nearest_idx];

  // 根据车辆的左右悬垂调整点的位置
  // shift nearest point on its local y axis so that vehicle's right and left edges
  // would have approx the same clearance from road border
  const auto shift_length = (vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  const auto delta_x = -shift_length * std::sin(lane_yaw);
  const auto delta_y = shift_length * std::cos(lane_yaw);

  lanelet::BasicPoint3d refined_point(
    nearest_point.x() + delta_x, nearest_point.y() + delta_y, nearest_point.z());

  // 将调整后的点转换为位姿
  return convertBasicPoint3dToPose(refined_point, lane_yaw);
}

}  // namespace autoware::mission_planner_universe::lanelet2
