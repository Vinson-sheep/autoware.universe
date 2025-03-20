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

#include "autoware/behavior_velocity_planner/planner_manager.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <boost/format.hpp>

#include <memory>
#include <optional>
#include <string>

namespace autoware::behavior_velocity_planner
{
  // BehaviorVelocityPlannerManager 构造函数
BehaviorVelocityPlannerManager::BehaviorVelocityPlannerManager()
: plugin_loader_(
    "autoware_behavior_velocity_planner", "autoware::behavior_velocity_planner::PluginInterface")
{
  // 初始化插件加载器，用于动态加载行为速度规划器的插件
}

// 启动场景插件
void BehaviorVelocityPlannerManager::launchScenePlugin(
  rclcpp::Node & node, const std::string & name)
{
  // 检查插件是否可用
  if (plugin_loader_.isClassAvailable(name)) {

    // 创建插件实例
    const auto plugin = plugin_loader_.createSharedInstance(name);
    plugin->init(node); // 初始化插件

    // 检查插件是否已经注册
    for (const auto & running_plugin : scene_manager_plugins_) {
      if (plugin->getModuleName() == running_plugin->getModuleName()) {
        RCLCPP_WARN_STREAM(node.get_logger(), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    // 注册插件
    scene_manager_plugins_.push_back(plugin);
    RCLCPP_DEBUG_STREAM(node.get_logger(), "The scene plugin '" << name << "' is loaded.");
  } else {
    RCLCPP_ERROR_STREAM(node.get_logger(), "The scene plugin '" << name << "' is not available.");
  }
}

// 移除场景插件
void BehaviorVelocityPlannerManager::removeScenePlugin(
  rclcpp::Node & node, const std::string & name)
{
  // 查找并移除指定名称的插件
  auto it = std::remove_if(
    scene_manager_plugins_.begin(), scene_manager_plugins_.end(),
    [&](const std::shared_ptr<behavior_velocity_planner::PluginInterface> plugin) {
      return plugin->getModuleName() == name;
    });

  if (it == scene_manager_plugins_.end()) {
    RCLCPP_WARN_STREAM(
      node.get_logger(),
      "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    scene_manager_plugins_.erase(it, scene_manager_plugins_.end());
    RCLCPP_INFO_STREAM(node.get_logger(), "The scene plugin '" << name << "' is unloaded.");
  }
}

// 规划路径速度
autoware_internal_planning_msgs::msg::PathWithLaneId
BehaviorVelocityPlannerManager::planPathVelocity(
  const std::shared_ptr<const PlannerData> & planner_data,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & input_path_msg)
{
  autoware_internal_planning_msgs::msg::PathWithLaneId output_path_msg = input_path_msg;

  // 遍历所有场景插件，更新场景模块实例并规划路径速度
  for (const auto & plugin : scene_manager_plugins_) {
    plugin->updateSceneModuleInstances(planner_data, input_path_msg);
    plugin->plan(&output_path_msg);
  }

  return output_path_msg;
}

}  // namespace autoware::behavior_velocity_planner
