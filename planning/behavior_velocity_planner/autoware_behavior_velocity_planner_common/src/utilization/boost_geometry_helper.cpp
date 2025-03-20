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

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>

#include <boost/geometry/algorithms/correct.hpp>

namespace autoware::behavior_velocity_planner
{

// 主要用于自动驾驶系统中的路径规划和环境建模模块。通过对多边形进行操作和转换，可以实现对车辆周围环境的建模和路径的生成。
// 例如，可以将道路边界表示为多边形，并对其进行缩放以适应不同的规划需求。

// 将两条线（左线和右线）转换为多边形
Polygon2d lines2polygon(const LineString2d & left_line, const LineString2d & right_line)
{
  Polygon2d polygon;  // 定义一个多边形

  polygon.outer().push_back(left_line.front()); // 将左线的第一个点加入多边形的外环

  // 遍历右线的所有点，并将其加入多边形的外环
  for (const auto & itr : right_line) {
    polygon.outer().push_back(itr);
  }

  // 遍历左线的所有点（从后往前），并将其加入多边形的外环
  for (auto itr = left_line.rbegin(); itr != left_line.rend(); ++itr) {
    polygon.outer().push_back(*itr);
  }

  bg::correct(polygon); // 调整多边形的几何形状，确保其有效性
  return polygon; // 返回生成的多边形
}

// 对多边形进行缩放
Polygon2d upScalePolygon(
  const geometry_msgs::msg::Point & position, const Polygon2d & polygon, const double scale)
{
  Polygon2d transformed_polygon;  // 定义一个变换后的多边形
  // upscale
  for (size_t i = 0; i < polygon.outer().size(); i++) {
    const double upscale_x = (polygon.outer().at(i).x() - position.x) * scale + position.x; // 计算缩放后的x坐标
    const double upscale_y = (polygon.outer().at(i).y() - position.y) * scale + position.y; // 计算缩放后的y坐标
    transformed_polygon.outer().emplace_back(Point2d(upscale_x, upscale_y));  // 将缩放后的点加入变换后的多边形
  }
  return transformed_polygon; // 返回缩放后的多边形
}

// 将Boost几何库的多边形转换为ROS消息格式
geometry_msgs::msg::Polygon toGeomPoly(const Polygon2d & polygon)
{
  geometry_msgs::msg::Polygon polygon_msg;  // 定义一个ROS多边形消息
  geometry_msgs::msg::Point32 point_msg;  // 定义一个ROS点消息
  for (const auto & p : polygon.outer()) {
    point_msg.x = static_cast<float>(p.x());  // 设置点的x坐标
    point_msg.y = static_cast<float>(p.y());  // 设置点的y坐标
    polygon_msg.points.push_back(point_msg);  // 将点加入ROS多边形消息
  }
  return polygon_msg; // 返回转换后的ROS多边形消息
}
}  // namespace autoware::behavior_velocity_planner
