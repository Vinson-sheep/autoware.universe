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

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/map_utils.hpp"

#include <boost/geometry/strategies/strategies.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::drivable_area_expansion
{

// 提取不可跨越的线段
SegmentRtree extract_uncrossable_segments(
  const lanelet::LaneletMap & lanelet_map, const Point & ego_point,
  const DrivableAreaExpansionParameters & params)
{
  SegmentRtree uncrossable_segments_in_range; // 用于存储范围内的不可跨越线段
  LineString2d line;  // 用于临时存储线段
  const auto ego_p = Point2d{ego_point.x, ego_point.y}; // 自车位置的二维点
  // 遍历 Lanelet 地图中的所有线串
  for (const auto & ls : lanelet_map.lineStringLayer) {
    // 检查线串是否属于需要避让的类型
    if (has_types(ls, params.avoid_linestring_types)) {
      line.clear(); // 清空临时线段
      // 将线串中的点转换为二维点
      for (const auto & p : ls) line.push_back(Point2d{p.x(), p.y()});
      // 遍历线串中的每一段线段
      for (auto segment_idx = 0LU; segment_idx + 1 < line.size(); ++segment_idx) {
        Segment2d segment = {line[segment_idx], line[segment_idx + 1]}; // 创建线段
        // 检查线段是否在自车位置的最大路径弧长范围内
        if (boost::geometry::distance(segment, ego_p) < params.max_path_arc_length) {
          uncrossable_segments_in_range.insert(segment);  // 将线段插入 R 树中
        }
      }
    }
  }
  return uncrossable_segments_in_range;
}

// 检查线串是否属于指定类型
bool has_types(const lanelet::ConstLineString3d & ls, const std::vector<std::string> & types)
{
  constexpr auto no_type = "";  // 默认无类型
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);  // 获取线串的类型属性
  // 检查类型是否在指定的类型列表中
  return (type != no_type && std::find(types.begin(), types.end(), type) != types.end());
}
}  // namespace autoware::behavior_path_planner::drivable_area_expansion
