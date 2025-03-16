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

/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#include "autoware/costmap_generator/utils/points_to_costmap.hpp"

#include <string>
#include <vector>

namespace autoware::costmap_generator
{

// 点云到成本图转换类

// 初始化网格地图参数
void PointsToCostmap::initGridmapParam(const grid_map::GridMap & gridmap)
{
  grid_length_x_ = gridmap.getLength().x(); // 网格地图的x方向长度
  grid_length_y_ = gridmap.getLength().y(); // 网格地图的y方向长度
  grid_resolution_ = gridmap.getResolution(); // 网格地图的分辨率
  grid_position_x_ = gridmap.getPosition().x(); // 网格地图的x方向位置
  grid_position_y_ = gridmap.getPosition().y(); // 网格地图的y方向位置
}

// 检查网格索引是否有效
bool PointsToCostmap::isValidInd(const grid_map::Index & grid_ind)
{
  bool is_valid = false;
  int x_grid_ind = grid_ind.x();  // x方向的网格索引
  int y_grid_ind = grid_ind.y();  // y方向的网格索引
  if (
    x_grid_ind >= 0 && x_grid_ind < std::ceil(grid_length_x_ * (1 / grid_resolution_)) &&
    y_grid_ind >= 0 && y_grid_ind < std::ceil(grid_length_y_ * (1 / grid_resolution_))) {
    is_valid = true;
  }
  return is_valid;
}

// 从点云中的点获取网格索引
grid_map::Index PointsToCostmap::fetchGridIndexFromPoint(const pcl::PointXYZ & point)
{
  // 计算点在网格地图中的位置
  const double origin_x_offset = grid_length_x_ / 2.0 - grid_position_x_; // x方向的偏移量
  const double origin_y_offset = grid_length_y_ / 2.0 - grid_position_y_; // y方向的偏移量
  /// 坐标转换，将点云中的点映射到网格地图的索引
  double mapped_x = (grid_length_x_ - origin_x_offset - point.x) / grid_resolution_;
  double mapped_y = (grid_length_y_ - origin_y_offset - point.y) / grid_resolution_;

  int mapped_x_ind = std::ceil(mapped_x); // x方向的网格索引
  int mapped_y_ind = std::ceil(mapped_y); // y方向的网格索引
  grid_map::Index index(mapped_x_ind, mapped_y_ind);  // 创建网格索引
  return index;
}

// 将点云中的点分配到网格单元
std::vector<std::vector<std::vector<double>>> PointsToCostmap::assignPoints2GridCell(
  const pcl::PointCloud<pcl::PointXYZ> & in_sensor_points)
{
  double y_cell_size = std::ceil(grid_length_y_ * (1 / grid_resolution_));  // y方向的网格单元数
  double x_cell_size = std::ceil(grid_length_x_ * (1 / grid_resolution_));  // x方向的网格单元数
  std::vector<double> z_vec;  // z方向的值
  std::vector<std::vector<double>> vec_y_z(y_cell_size, z_vec); // y方向的网格单元
  std::vector<std::vector<std::vector<double>>> vec_x_y_z(x_cell_size, vec_y_z);  // x方向的网格单元

  for (const auto & point : in_sensor_points) { // 遍历点云中的每个点
    grid_map::Index grid_ind = fetchGridIndexFromPoint(point);  // 获取点的网格索引
    if (isValidInd(grid_ind)) { // 如果网格索引有效
      vec_x_y_z[grid_ind.x()][grid_ind.y()].push_back(point.z); // 将点的z值添加到对应的网格单元
    }
  }
  return vec_x_y_z;
}

// 计算成本图
grid_map::Matrix PointsToCostmap::calculateCostmap(
  const double maximum_height_thres, const double minimum_lidar_height_thres,
  const double grid_min_value, const double grid_max_value, const grid_map::GridMap & gridmap,
  const std::string & gridmap_layer_name,
  const std::vector<std::vector<std::vector<double>>> grid_vec)
{
  grid_map::Matrix gridmap_data = gridmap[gridmap_layer_name];  // 获取网格地图的数据
  for (size_t x_ind = 0; x_ind < grid_vec.size(); x_ind++) {  // 遍历x方向的网格单元
    for (size_t y_ind = 0; y_ind < grid_vec[0].size(); y_ind++) { // 遍历y方向的网格单元
      if (grid_vec[x_ind][y_ind].size() == 0) { // 如果网格单元中没有点
        gridmap_data(x_ind, y_ind) = grid_min_value;  // 设置为最小值
        continue;
      }
      for (const auto & z : grid_vec[x_ind][y_ind]) { // 遍历网格单元中的点
        if (z > maximum_height_thres || z < minimum_lidar_height_thres) { // 如果点的高度超出阈值
          continue;
        }
        gridmap_data(x_ind, y_ind) = grid_max_value;  // 设置为最大值
        break;
      }
    }
  }
  return gridmap_data;
}

// 从点云生成成本图
grid_map::Matrix PointsToCostmap::makeCostmapFromPoints(
  const double maximum_height_thres, const double minimum_lidar_height_thres,
  const double grid_min_value, const double grid_max_value, const grid_map::GridMap & gridmap,
  const std::string & gridmap_layer_name, const pcl::PointCloud<pcl::PointXYZ> & in_sensor_points)
{
  initGridmapParam(gridmap);  // 初始化网格地图参数
  std::vector<std::vector<std::vector<double>>> grid_vec = assignPoints2GridCell(in_sensor_points); // 将点云中的点分配到网格单元
  grid_map::Matrix costmap = calculateCostmap(
    maximum_height_thres, minimum_lidar_height_thres, grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name, grid_vec);  // 计算成本图
  return costmap;
}

}  // namespace autoware::costmap_generator
