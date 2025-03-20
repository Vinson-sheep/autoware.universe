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

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <string>
#include <vector>
namespace autoware::behavior_velocity_planner::debug
{
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;

// 创建多边形的MarkerArray
visualization_msgs::msg::MarkerArray createPolygonMarkerArray(
  const geometry_msgs::msg::Polygon & polygon, const std::string & ns, const int64_t module_id,
  const rclcpp::Time & now, const double x, const double y, const double z, const double r,
  const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg; // 定义一个MarkerArray消息
  {
    auto marker = create_default_marker(
      "map", now, ns, static_cast<int32_t>(module_id), visualization_msgs::msg::Marker::LINE_STRIP,
      create_marker_scale(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)),
      create_marker_color(
        static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.8f));  // 创建一个默认的Marker
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);  // 设置Marker的生命周期

    for (const auto & p : polygon.points) { // 遍历多边形的每个点
      geometry_msgs::msg::Point point;  // 定义一个点
      point.x = p.x;  // 设置点的x坐标
      point.y = p.y;  // 设置点的y坐标
      point.z = p.z;  // 设置点的z坐标
      marker.points.push_back(point); // 将点加入Marker
    }

    if (!marker.points.empty()) { // 如果Marker中有点
      marker.points.push_back(marker.points.front()); // 将第一个点加入，闭合多边形
    }
    msg.markers.push_back(marker);  // 将Marker加入MarkerArray
  }
  return msg; // 返回MarkerArray
}

// 创建路径的MarkerArray
visualization_msgs::msg::MarkerArray createPathMarkerArray(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const std::string & ns,
  const int64_t lane_id, const rclcpp::Time & now, const double x, const double y, const double z,
  const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg; // 定义一个MarkerArray消息

  for (size_t i = 0; i < path.points.size(); ++i) { // 遍历路径的每个点
    const auto & p = path.points.at(i); // 获取路径点

    auto marker = create_default_marker(
      "map", now, ns, static_cast<int32_t>(planning_utils::bitShift(lane_id) + i),
      visualization_msgs::msg::Marker::ARROW,
      create_marker_scale(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)),
      create_marker_color(
        static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.999f));  // 创建一个默认的Marker
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);  // 设置Marker的生命周期
    marker.pose = p.point.pose; // 设置Marker的位姿

    if (std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id) != p.lane_ids.end()) {
      // 如果路径点的车道ID包含指定的车道ID
      marker.color = create_marker_color(
        static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.999f); // 设置Marker的颜色
    } else {
      marker.color = create_marker_color(0.5, 0.5, 0.5, 0.999); // 设置默认颜色
    }
    msg.markers.push_back(marker);  // 将Marker加入MarkerArray
  }

  return msg;
}

// 创建目标对象的MarkerArray
visualization_msgs::msg::MarkerArray createObjectsMarkerArray(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const std::string & ns,
  const int64_t module_id, const rclcpp::Time & now, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg; // 定义一个MarkerArray消息

  auto marker = create_default_marker(
    "map", now, ns, 0, visualization_msgs::msg::Marker::CUBE, create_marker_scale(3.0, 1.0, 1.0),
    create_marker_color(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.8f));  // 创建一个默认的Marker
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);  // 设置Marker的生命周期

  for (size_t i = 0; i < objects.objects.size(); ++i) { // 遍历目标对象
    const auto & object = objects.objects.at(i);  // 获取目标对象

    marker.id = static_cast<int>(planning_utils::bitShift(module_id) + i);  // 设置Marker的ID
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;  // 设置Marker的位姿
    msg.markers.push_back(marker);  // 将Marker加入MarkerArray
  }

  return msg; // 返回MarkerArray
}

// 创建点的MarkerArray
visualization_msgs::msg::MarkerArray createPointsMarkerArray(
  const std::vector<geometry_msgs::msg::Point> & points, const std::string & ns,
  const int64_t module_id, const rclcpp::Time & now, const double x, const double y, const double z,
  const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg; // 定义一个MarkerArray消息

  auto marker = create_default_marker(
    "map", now, ns, 0, visualization_msgs::msg::Marker::SPHERE, create_marker_scale(x, y, z),
    create_marker_color(
      static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.999f));  // 创建一个默认的Marker
  for (size_t i = 0; i < points.size(); ++i) {
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);  
  for (size_t i = 0; i < points.size(); ++i) {  // 遍历点
    marker.id = static_cast<int32_t>(i + planning_utils::bitShift(module_id));  // 设置Marker的ID
    marker.pose.position = points.at(i);  // 设置Marker的位置
    msg.markers.push_back(marker);  // 将Marker加入MarkerArray
  }

  return msg; // 返回MarkerArray
}
}  // namespace autoware::behavior_velocity_planner::debug
