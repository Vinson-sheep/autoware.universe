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

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/footprints.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

#include <tf2/utils.h>

namespace autoware::behavior_path_planner::drivable_area_expansion
{

// 平移多边形
Polygon2d translate_polygon(const Polygon2d & polygon, const double x, const double y)
{
  Polygon2d translated_polygon;
  const boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translation(x, y);
  boost::geometry::transform(polygon, translated_polygon, translation);
  return translated_polygon;
}

// 创建足迹多边形
Polygon2d create_footprint(const geometry_msgs::msg::Pose & pose, const Polygon2d & base_footprint)
{
  // 获取姿态的偏航角
  const auto angle = tf2::getYaw(pose.orientation);
  // 先旋转多边形，再平移
  return translate_polygon(
    autoware_utils::rotate_polygon(base_footprint, angle), pose.position.x, pose.position.y);
}

// 创建动态物体的足迹
MultiPolygon2d create_object_footprints(
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const DrivableAreaExpansionParameters & params)
{
  MultiPolygon2d footprints;
  // 如果启用了避让动态物体
  if (params.avoid_dynamic_objects) {
    for (const auto & object : objects.objects) {
      // 计算物体的前后左右边界，考虑额外的偏移量
      const auto front = object.shape.dimensions.x / 2 + params.dynamic_objects_extra_front_offset;
      const auto rear = -object.shape.dimensions.x / 2 - params.dynamic_objects_extra_rear_offset;
      const auto left = object.shape.dimensions.y / 2 + params.dynamic_objects_extra_left_offset;
      const auto right = -object.shape.dimensions.y / 2 - params.dynamic_objects_extra_right_offset;
      // 创建基础足迹多边形
      Polygon2d base_footprint;
      base_footprint.outer() = {
        Point2d{front, left}, Point2d{front, right}, Point2d{rear, right}, Point2d{rear, left},
        Point2d{front, left}};
      // 遍历物体的预测路径，为每个姿态创建足迹
      for (const auto & path : object.kinematics.predicted_paths)
        for (const auto & pose : path.path)
          footprints.push_back(create_footprint(pose, base_footprint));
    }
  }
  return footprints;
}
}  // namespace autoware::behavior_path_planner::drivable_area_expansion
