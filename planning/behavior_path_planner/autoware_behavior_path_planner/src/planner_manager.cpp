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

#include "autoware/behavior_path_planner/planner_manager.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware_utils/ros/debug_publisher.hpp"
#include "autoware_utils/system/stop_watch.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <magic_enum.hpp>

#include <boost/scope_exit.hpp>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

// PlannerManager 构造函数
PlannerManager::PlannerManager(rclcpp::Node & node)
: plugin_loader_(
    "autoware_behavior_path_planner",
    "autoware::behavior_path_planner::SceneModuleManagerInterface"),
  logger_(node.get_logger().get_child("planner_manager")),
  clock_(*node.get_clock())
{
  // 初始化当前路由车
  current_route_lanelet_ = std::make_shared<std::optional<lanelet::ConstLanelet>>(std::nullopt);
  processing_time_.emplace("total_time", 0.0);
  debug_publisher_ptr_ = std::make_unique<DebugPublisher>(&node, "~/debug");
  state_publisher_ptr_ = std::make_unique<DebugPublisher>(&node, "~/debug");
}

// 加载场景插件
void PlannerManager::launchScenePlugin(rclcpp::Node & node, const std::string & name)
{
  if (plugin_loader_.isClassAvailable(name)) {  // 检查插件是否可用
    const auto plugin = plugin_loader_.createSharedInstance(name);  // 创建插件实例
    plugin->init(&node);  // 初始化插件

     // 检查插件是否已注册
    // Check if the plugin is already registered.
    for (const auto & running_plugin : manager_ptrs_) {
      if (plugin->name() == running_plugin->name()) {
        RCLCPP_WARN_STREAM(node.get_logger(), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    // 注册插件
    // register
    manager_ptrs_.push_back(plugin);  // 将插件添加到管理器列表
    processing_time_.emplace(plugin->name(), 0.0);  // 初始化插件的处理时间
    RCLCPP_DEBUG_STREAM(node.get_logger(), "The scene plugin '" << name << "' is loaded.");
  } else {
    RCLCPP_ERROR_STREAM(node.get_logger(), "The scene plugin '" << name << "' is not available.");
  }
}

// 配置模块插槽
void PlannerManager::configureModuleSlot(
  const std::vector<std::vector<std::string>> & slot_configuration)
{
  // 创建已注册模块的映射
  std::unordered_map<std::string, SceneModuleManagerPtr> registered_modules;
  for (const auto & manager_ptr : manager_ptrs_) {
    registered_modules[manager_ptr->name()] = manager_ptr;  // 注册所有已加载的模块
  }

  // 为每个槽配置创建一个SubPlannerManager
  for (const auto & slot : slot_configuration) {
    SubPlannerManager sub_manager(current_route_lanelet_, processing_time_, debug_info_);
    // 将配置中指定的模块添加到这个槽中
    for (const auto & module_name : slot) {
      if (const auto it = registered_modules.find(module_name); it != registered_modules.end()) {
        sub_manager.addSceneModuleManager(it->second);
      } 
      // else {
      //   // TODO(Mamoru Sobue): use LOG
      //   std::cout << module_name << " registered in slot_configuration is not registered, skipping"
      //             << std::endl;
      // }
    }
    // 如果槽中有模块，则添加到planner_manager_slots_中
    if (sub_manager.getSceneModuleManager().size() != 0) {
      planner_manager_slots_.push_back(sub_manager);
      // TODO(Mamoru Sobue): use LOG
      // std::cout << "added a slot with " << sub_manager.getSceneModuleManager().size() << " modules"
      //           << std::endl;
    }
  }
}

// 执行路径规划
BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
  resetProcessingTime();  // 重置处理时间
  StopWatch<std::chrono::milliseconds> stop_watch;  // 启动计时器
  stop_watch.tic("total_time");
  BOOST_SCOPE_EXIT((&processing_time_)(&stop_watch))
  {
    processing_time_.at("total_time") += stop_watch.toc("total_time", true);  // 记录总处理时间
  }
  BOOST_SCOPE_EXIT_END;

  debug_info_.scene_status.clear(); // 清空调试信息
  debug_info_.slot_status.clear();

  // 重置当前路由车道 (取最近车道)
  if (!current_route_lanelet_->has_value()) resetCurrentRouteLanelet(data);

  // 为场景模块注入数据
  std::for_each(
    manager_ptrs_.begin(), manager_ptrs_.end(), [&data](const auto & m) { m->setData(data); });

  // 检查是否有已批准模块正在运行
  const bool is_any_approved_module_running = std::any_of(
    planner_manager_slots_.begin(), planner_manager_slots_.end(), [&](const auto & slot) {
      return slot.isAnyApprovedPred([](const auto & m) {
        const auto status = m->getCurrentStatus();
        return status == ModuleStatus::RUNNING || status == ModuleStatus::WAITING_APPROVAL;
      });
    });

  // 检查是否有候选模块正在运行或空闲
  // IDLE is a state in which an execution has been requested but not yet approved.
  // once approved, it basically turns to running.
  const bool is_any_candidate_module_running_or_idle = std::any_of(
    planner_manager_slots_.begin(), planner_manager_slots_.end(), [](const auto & slot) {
      return slot.isAnyCandidatePred([](const auto & m) {
        const auto status = m->getCurrentStatus();
        return status == ModuleStatus::RUNNING || status == ModuleStatus::WAITING_APPROVAL ||
               status == ModuleStatus::IDLE;
      });
    });

  // 检查是否有模块正在运行
  const bool is_any_module_running =
    is_any_approved_module_running || is_any_candidate_module_running_or_idle;

  // 更新当前路由车道
  updateCurrentRouteLanelet(data, is_any_approved_module_running);

  // 检查车辆是否超出路由范围
  const bool is_out_of_route = utils::isEgoOutOfRoute(
    data->self_odometry->pose.pose, current_route_lanelet_->value(), data->prev_modified_goal,
    data->route_handler); 

  // 如果没有模块正在运行且车辆不在路径内，则跳过场景模块的运行
  if (!is_any_module_running && is_out_of_route) {
    // 生成在目标点附近的刹车路径
    BehaviorModuleOutput result_output = utils::createGoalAroundPath(data);
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 5000,
      "Ego is out of route, no module is running. Skip running scene modules.");
    generateCombinedDrivableArea(result_output, data);  // 生成可行驶区域
    return result_output;
  }

  // 初始化结果输出
  SlotOutput result_output = SlotOutput{
    getReferencePath(data), // 获取参考路径
    false,  // 是否上游批准失败
    false,  // 是否上游等待批准
    false,  // 是否上游候选独占
  };

  // 逐个运行场景模块
  for (auto & planner_manager_slot : planner_manager_slots_) {
    if (result_output.is_upstream_failed_approved) {
      // 如果上游模块批准失败，则清除后续模块的候选/批准状态
      planner_manager_slot.propagateWithFailedApproved();
      debug_info_.slot_status.push_back(SlotStatus::UPSTREAM_APPROVED_FAILED);
    } else if (result_output.is_upstream_waiting_approved) {
      // 如果上游模块等待批准，则处理当前模块
      result_output = planner_manager_slot.propagateWithWaitingApproved(data, result_output);
      debug_info_.slot_status.push_back(SlotStatus::UPSTREAM_WAITING_APPROVED);
    } else if (result_output.is_upstream_candidate_exclusive) {
      // 如果上游模块是候选独占，则处理当前模块
      result_output = planner_manager_slot.propagateWithExclusiveCandidate(data, result_output);
      debug_info_.slot_status.push_back(SlotStatus::UPSTREAM_EXCLUSIVE_CANDIDATE);
    } else {
      // 正常运行当前模块
      result_output = planner_manager_slot.propagateFull(data, result_output);
      debug_info_.slot_status.push_back(SlotStatus::NORMAL);
    }
  }

  // 更新模块状态并发布 RTC 状态
  std::for_each(manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) {
    m->updateObserver();
    m->publishRTCStatus();
    m->publish_planning_factors();
  });

  // 生成合并的可行驶区域
  generateCombinedDrivableArea(result_output.valid_output, data);
  return result_output.valid_output;  // 返回有效的路径规划结果
}

// 生成合并的可行驶区域
// NOTE: To deal with some policies about drivable area generation, currently DrivableAreaInfo is
// quite messy. Needs to be refactored.
void PlannerManager::generateCombinedDrivableArea(
  BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & data) const
{
  // 检查路径是否为空
  if (output.path.points.empty()) { 
    RCLCPP_ERROR_STREAM(logger_, "[generateCombinedDrivableArea] Output path is empty!");
    return;
  }

  const auto & di = output.drivable_area_info;
  constexpr double epsilon = 1e-3;

  const auto is_driving_forward_opt = autoware::motion_utils::isDrivingForward(output.path.points);
  const bool is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  // 如果存在可行驶区域边距
  if (epsilon < std::abs(di.drivable_margin)) {
    // for single free space pull over
    utils::generateDrivableArea(
      output.path, data->parameters.vehicle_length, di.drivable_margin, is_driving_forward);  // 生成可行驶区域
  } else if (di.is_already_expanded) {  // 如果可行驶区域已扩展
    // for single side shift
    utils::generateDrivableArea(
      output.path, di.drivable_lanes, false, false, false, data, is_driving_forward);
  } else {
    const auto shorten_lanes = utils::cutOverlappedLanes(output.path, di.drivable_lanes); // 裁剪重叠车道

    const auto & dp = data->drivable_area_expansion_parameters;
    const auto expanded_lanes = utils::expandLanelets(
      shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
      dp.drivable_area_types_to_skip);  // 扩展车道

    // for other modules where multiple modules may be launched
    utils::generateDrivableArea(
      output.path, expanded_lanes, di.enable_expanding_hatched_road_markings,
      di.enable_expanding_intersection_areas, di.enable_expanding_freespace_areas, data,
      is_driving_forward); // 生成可行驶区域
  }

  // extract obstacles from drivable area
  utils::extractObstaclesFromDrivableArea(output.path, di.obstacles);   // 从可行驶区域中提取障碍物
}

// 更新当前路由车道
void PlannerManager::updateCurrentRouteLanelet(
  const std::shared_ptr<PlannerData> & data, const bool is_any_approved_module_running)
{
  const auto & route_handler = data->route_handler;
  const auto & pose = data->self_odometry->pose.pose;
  const auto p = data->parameters;

  constexpr double extra_margin = 10.0;
  const auto backward_length =
    std::max(p.backward_path_length, p.backward_path_length + extra_margin);

  lanelet::ConstLanelet closest_lane{};

  // 如果当前lanelet就是最近的，直接使用
  if (route_handler->getClosestRouteLaneletFromLanelet(
        pose, current_route_lanelet_->value(), &closest_lane, p.ego_nearest_dist_threshold,
        p.ego_nearest_yaw_threshold)) { // 获取最近的车道
    *current_route_lanelet_ = closest_lane;
    return;
  }

  // 从后续的lanelat中选择最近的
  const auto lanelet_sequence = route_handler->getLaneletSequence(
    current_route_lanelet_->value(), pose, backward_length, p.forward_path_length); // 获取车道序列

  const auto could_calculate_closest_lanelet =
    lanelet::utils::query::getClosestLaneletWithConstrains(
      lanelet_sequence, pose, &closest_lane, p.ego_nearest_dist_threshold,
      p.ego_nearest_yaw_threshold) || // 获取最近的车道
    lanelet::utils::query::getClosestLanelet(lanelet_sequence, pose, &closest_lane);

  // 如果找到，直接使用
  if (could_calculate_closest_lanelet) {
    *current_route_lanelet_ = closest_lane;
  } else if (!is_any_approved_module_running) {
    resetCurrentRouteLanelet(data); // 重置当前路由车道
  }
}

// 获取参考路径
BehaviorModuleOutput PlannerManager::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  const auto reference_path = utils::getReferencePath(current_route_lanelet_->value(), data); // 获取参考路径
  publishDebugRootReferencePath(reference_path);  // 发布调试信息
  return reference_path;
}

// 发布调试参考路径
void PlannerManager::publishDebugRootReferencePath(
  const BehaviorModuleOutput & reference_path) const
{
  using visualization_msgs::msg::Marker;
  MarkerArray array;
  Marker m = autoware_utils::create_default_marker(
    "map", clock_.now(), "root_reference_path", 0UL, Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(1.0, 1.0, 1.0),
    autoware_utils::create_marker_color(1.0, 0.0, 0.0, 1.0));
  for (const auto & p : reference_path.path.points) m.points.push_back(p.point.pose.position);
  array.markers.push_back(m);
  m.points.clear();
  m.id = 1UL;
  for (const auto & p : current_route_lanelet_->value().polygon3d().basicPolygon())
    m.points.emplace_back().set__x(p.x()).set__y(p.y()).set__z(p.z());
  array.markers.push_back(m);
  debug_publisher_ptr_->publish<MarkerArray>("root_reference_path", array); // 发布参考路径
}

// 检查是否有可重新路由的已批准模块
bool PlannerManager::hasPossibleRerouteApprovedModules(
  const std::shared_ptr<PlannerData> & data) const
{
  const auto & approved_module = approved_modules();
  const auto not_possible_reroute_module = [&](const SceneModulePtr m) {
    if (m->name() == "dynamic_avoidance") {
      return false;
    }
    if (m->name() == "goal_planner" && !utils::isAllowedGoalModification(data->route_handler)) {
      return false;
    }
    return true;
  };

  return std::any_of(approved_module.begin(), approved_module.end(), not_possible_reroute_module);
}

// 打印规划器状态
void PlannerManager::print() const
{
  const auto approved_module_ptrs = approved_modules();
  const auto candidate_module_ptrs = candidate_modules();

  const auto get_status = [](const auto & m) {
    return magic_enum::enum_name(m->getCurrentStatus());
  };

  size_t max_string_num = 0;

  std::ostringstream string_stream;
  string_stream << "\n";
  string_stream << "***********************************************************\n";
  string_stream << "                  planner manager status\n";
  string_stream << "-----------------------------------------------------------\n";
  string_stream << "registered modules: ";
  for (const auto & m : manager_ptrs_) {
    string_stream << "[" << m->name() << "]";
    max_string_num = std::max(max_string_num, m->name().length());
  }

  string_stream << "\n";
  string_stream << "approved modules  : ";
  std::string delimiter = "";
  for (const auto & planner_manager_slot : planner_manager_slots_) {
    string_stream << std::exchange(delimiter, " ==> ") << "[[ ";
    std::string delimiter_sub = "";
    for (const auto & m : planner_manager_slot.approved_modules()) {
      string_stream << std::exchange(delimiter_sub, "->") << "[" << m->name() << "("
                    << get_status(m) << ")"
                    << "]";
    }
    string_stream << " ]]";
  }

  string_stream << "\n";
  string_stream << "candidate modules : ";
  delimiter = "";
  for (const auto & planner_manager_slot : planner_manager_slots_) {
    string_stream << std::exchange(delimiter, " ==> ") << "[[ ";
    std::string delimiter_sub = "";
    for (const auto & m : planner_manager_slot.candidate_modules()) {
      string_stream << std::exchange(delimiter_sub, "->") << "[" << m->name() << "("
                    << get_status(m) << ")"
                    << "]";
    }
    string_stream << " ]]";
  }

  string_stream << "\n";
  string_stream << "slot_status       : ";
  delimiter = "";
  for (const auto & slot_status : debug_info_.slot_status) {
    string_stream << std::exchange(delimiter, "->") << "[" << magic_enum::enum_name(slot_status)
                  << "]";
  }

  string_stream << "\n";
  string_stream << "update module info: ";
  for (const auto & i : debug_info_.scene_status) {
    string_stream << "[Module:" << i.module_name << " Status:" << magic_enum::enum_name(i.status)
                  << " Action:" << magic_enum::enum_name(i.action)
                  << " Description:" << i.description << "]\n"
                  << std::setw(28);
  }

  string_stream << "\n" << std::fixed << std::setprecision(1);
  string_stream << "processing time   : ";
  for (const auto & t : processing_time_) {
    string_stream << std::right << "[" << std::setw(static_cast<int>(max_string_num) + 1)
                  << std::left << t.first << ":" << std::setw(4) << std::right << t.second
                  << "ms]\n"
                  << std::setw(21);
  }

  state_publisher_ptr_->publish<DebugStringMsg>("internal_state", string_stream.str());

  RCLCPP_DEBUG_STREAM(logger_, string_stream.str());
}

void PlannerManager::publishProcessingTime() const
{
  for (const auto & t : processing_time_) {
    std::string name = t.first + std::string("/processing_time_ms");
    debug_publisher_ptr_->publish<DebugDoubleMsg>(name, t.second);
  }
}

// 获取调试信息
std::shared_ptr<SceneModuleVisitor> PlannerManager::getDebugMsg()
{
  debug_msg_ptr_ = std::make_shared<SceneModuleVisitor>();
  const auto approved_module_ptrs = approved_modules();
  const auto candidate_module_ptrs = candidate_modules();

  for (const auto & approved_module : approved_module_ptrs) {
    approved_module->acceptVisitor(debug_msg_ptr_);
  }

  for (const auto & candidate_module : candidate_module_ptrs) {
    candidate_module->acceptVisitor(debug_msg_ptr_);
  }
  return debug_msg_ptr_;
}

std::vector<SceneModulePtr> SubPlannerManager::getRequestModules(
  const BehaviorModuleOutput & previous_module_output,
  const std::vector<SceneModulePtr> & deleted_modules) const
{
  std::vector<SceneModulePtr> request_modules{};
  StopWatch<std::chrono::milliseconds> stop_watch;

  for (const auto & manager_ptr : manager_ptrs_) {
    stop_watch.tic(manager_ptr->name());
    BOOST_SCOPE_EXIT((&manager_ptr)(&processing_time_)(&stop_watch))
    {
      processing_time_.at(manager_ptr->name()) += stop_watch.toc(manager_ptr->name(), true);
    }
    BOOST_SCOPE_EXIT_END;

    if (const auto deleted_it = std::find_if(
          deleted_modules.begin(), deleted_modules.end(),
          [&](const auto & m) { return m->name() == manager_ptr->name(); });
        deleted_it != deleted_modules.end()) {
      continue;
    }

    // Condition 1:
    // the approved module queue is either
    // - consists of multiple simultaneous_executable_as_approved modules only
    // - consists of only 1 "not simultaneous_executable_as_approved" module
    const bool exclusive_module_exist_in_approved_pool = std::any_of(
      approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const SceneModulePtr & m) {
        return !getManager(m)->isSimultaneousExecutableAsApprovedModule();
      });
    const bool is_this_exclusive = !manager_ptr->isSimultaneousExecutableAsApprovedModule();
    const bool approved_pool_is_not_empty = !approved_module_ptrs_.empty();

    const bool is_this_not_joinable =
      exclusive_module_exist_in_approved_pool || (is_this_exclusive && approved_pool_is_not_empty);
    if (is_this_not_joinable) {
      continue;
    }

    /**
     * launch new candidate module.
     */
    {
      const auto name = manager_ptr->name();
      const auto find_same_name_module = [&name](const auto & m) { return m->name() == name; };
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), find_same_name_module);

      if (itr == candidate_module_ptrs_.end()) {
        if (manager_ptr->canLaunchNewModule()) {
          manager_ptr->updateIdleModuleInstance();
          if (manager_ptr->isExecutionRequested(previous_module_output)) {
            request_modules.emplace_back(manager_ptr->getIdleModule());
          }
        }
        continue;
      }
    }

    /**
     * module already exist in candidate modules. check whether other modules can be launch as
     * candidate. if locked, break this loop.
     */
    {
      const auto name = manager_ptr->name();
      const auto find_block_module = [&name](const auto & m) {
        return m->name() == name && m->isLockedNewModuleLaunch();
      };
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), find_block_module);

      if (itr != candidate_module_ptrs_.end()) {
        request_modules.clear();
        request_modules.emplace_back(*itr);
        break;
      }
    }

    /**
     * module already exist. keep using it as candidate.
     */
    {
      const auto name = manager_ptr->name();
      const auto find_launched_module = [&name](const auto & m) { return m->name() == name; };
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), find_launched_module);

      if (itr != candidate_module_ptrs_.end()) {
        request_modules.emplace_back(*itr);
        continue;
      }
    }

    /**
     * same name module doesn't exist in candidate modules. launch new module.
     */
    {
      if (!manager_ptr->canLaunchNewModule()) {
        continue;
      }

      manager_ptr->updateIdleModuleInstance();
      if (!manager_ptr->isExecutionRequested(previous_module_output)) {
        continue;
      }

      request_modules.emplace_back(manager_ptr->getIdleModule());
    }
  }

  return request_modules;
}

SceneModulePtr SubPlannerManager::selectHighestPriorityModule(
  std::vector<SceneModulePtr> & request_modules) const
{
  if (request_modules.empty()) {
    return {};
  }

  sortByPriority(request_modules);

  return request_modules.front();
}

void SubPlannerManager::clearApprovedModules()
{
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [this](auto & m) {
    debug_info_.scene_status.emplace_back(
      m, SceneModuleUpdateInfo::Action::DELETE, "From Approved");
    deleteExpiredModules(m);
  });
  approved_module_ptrs_.clear();
}

void SubPlannerManager::updateCandidateModules(
  const std::vector<SceneModulePtr> & request_modules,
  const SceneModulePtr & highest_priority_module)
{
  const auto exist = [](const auto & module_ptr, const auto & module_ptrs) {
    const auto itr = std::find_if(
      module_ptrs.begin(), module_ptrs.end(),
      [&module_ptr](const auto & m) { return m->name() == module_ptr->name(); });

    return itr != module_ptrs.end();
  };

  /**
   * unregister expired modules
   */
  {
    const auto candidate_to_remove = [&](auto & itr) {
      if (!exist(itr, request_modules)) {
        deleteExpiredModules(itr);
        return true;
      }
      return itr->name() == highest_priority_module->name() &&
             !highest_priority_module->isWaitingApproval();
    };

    candidate_module_ptrs_.erase(
      std::remove_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), candidate_to_remove),
      candidate_module_ptrs_.end());

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  /**
   * register running candidate modules
   */
  for (const auto & m : request_modules) {
    if (
      m->name() == highest_priority_module->name() &&
      !highest_priority_module->isWaitingApproval()) {
      continue;
    }

    if (!exist(m, candidate_module_ptrs_)) {
      candidate_module_ptrs_.push_back(m);
    }
  }

  /**
   * sort by priority. sorted_request_modules.front() is the highest priority module.
   */
  sortByPriority(candidate_module_ptrs_);
}

// runRequestModules 函数：运行请求模块，并根据优先级选择一个模块的输出
std::pair<SceneModulePtr, BehaviorModuleOutput> SubPlannerManager::runRequestModules(
  const std::vector<SceneModulePtr> & request_modules, const std::shared_ptr<PlannerData> & data,
  const BehaviorModuleOutput & previous_module_output)
{
  // 用于存储满足同时执行条件的模块
  std::vector<SceneModulePtr> executable_modules;

  // 用于存储尚未获得批准的模块
  std::vector<SceneModulePtr> waiting_approved_modules;

  // 用于存储已经获得批准的模块
  std::vector<SceneModulePtr> already_approved_modules;

  // 用于存储所有请求模块的规划结果
  std::unordered_map<std::string, BehaviorModuleOutput> results;

  /**
   * 按优先级对请求模块进行排序。
   * sorted_request_modules.front() 是优先级最高的模块。
   */
  auto sorted_request_modules = request_modules;
  sortByPriority(sorted_request_modules);

  // 候选模块队列的组成规则：
  // - 仅包含多个满足同时执行条件的模块（SimultaneousExecutableAsCandidate）
  // - 或者仅包含一个不满足同时执行条件的模块
  for (const auto & module_ptr : sorted_request_modules) {
    // 如果 executable_modules 为空，任何模块都可以加入
    const bool is_executable_modules_empty = executable_modules.empty();
    if (is_executable_modules_empty) {
      executable_modules.push_back(module_ptr);
      continue;
    }

    // 如果 executable_modules 不为空，只有满足同时执行条件的模块才能加入
    const bool is_this_cooperative =
      getManager(module_ptr)->isSimultaneousExecutableAsCandidateModule();
    const bool any_other_cooperative = std::any_of(
      executable_modules.begin(), executable_modules.end(), [&](const SceneModulePtr & m) {
        return getManager(m)->isSimultaneousExecutableAsCandidateModule();
      });

    if (is_this_cooperative && any_other_cooperative) {
      executable_modules.push_back(module_ptr);
    }
  }

  /**
   * 运行可执行模块。
   */
  for (const auto & module_ptr : executable_modules) {
    const auto & manager_ptr = getManager(module_ptr);

    // 如果模块尚未注册，则注册新模块
    if (!manager_ptr->exist(module_ptr)) {
      manager_ptr->registerNewModule(
        std::weak_ptr<SceneModuleInterface>(module_ptr), previous_module_output);
    }

    // 运行模块并存储结果
    results.emplace(module_ptr->name(), run(module_ptr, data, previous_module_output));
  }

  /**
   * 删除过期的模块。
   */
  {
    // 定义删除过期模块的逻辑
    const auto remove_expired_modules = [this](auto & m) {
      if (m->getCurrentStatus() == ModuleStatus::FAILURE) {
        deleteExpiredModules(m);
        return true;
      }

      if (m->getCurrentStatus() == ModuleStatus::SUCCESS) {
        deleteExpiredModules(m);
        return true;
      }

      return false;
    };

    // 删除 executable_modules 中的过期模块
    executable_modules.erase(
      std::remove_if(executable_modules.begin(), executable_modules.end(), remove_expired_modules),
      executable_modules.end());


    // 更新所有管理器的观察者
    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  /**
   * 如果没有有效的候选模块，则返回空数据。
   */
  if (executable_modules.empty()) {
    clearCandidateModules();
    return std::make_pair(nullptr, BehaviorModuleOutput{});
  }

  /**
   * 根据批准条件将模块分为两类：
   * - 等待批准的模块
   * - 已批准的模块
   */
  std::for_each(executable_modules.begin(), executable_modules.end(), [&](const auto & m) {
    if (m->isWaitingApproval()) {
      waiting_approved_modules.push_back(m);
    } else {
      already_approved_modules.push_back(m);
    }
  });

  /**
   * 选择优先级最高的模块。
   */
  const auto module_ptr = [&]() -> SceneModulePtr {
    if (!already_approved_modules.empty()) {
      // 如果有已批准的模块，选择优先级最高的已批准模块
      return selectHighestPriorityModule(already_approved_modules);
    }

    if (!waiting_approved_modules.empty()) {
      // 如果有等待批准的模块，选择优先级最高的等待批准模块
      return selectHighestPriorityModule(waiting_approved_modules);
    }

    // 如果没有模块，返回 nullptr
    return nullptr;
  }();
  if (module_ptr == nullptr) {
    return std::make_pair(nullptr, BehaviorModuleOutput{});
  }

  /**
   * 注册候选模块。
   */
  updateCandidateModules(executable_modules, module_ptr);

  // 返回优先级最高的模块及其输出
  return std::make_pair(module_ptr, results.at(module_ptr->name()));
}

// SubPlannerManager 的 run 函数：运行一个场景模块并返回其输出结果
BehaviorModuleOutput SubPlannerManager::run(
  const SceneModulePtr & module_ptr, const std::shared_ptr<PlannerData> & planner_data,
  const BehaviorModuleOutput & previous_module_output) const
{
  // 创建一个用于测量时间的计时器，单位为毫秒
  StopWatch<std::chrono::milliseconds> stop_watch;
  // 开始计时，并以模块名称作为计时标签
  stop_watch.tic(module_ptr->name());

  // 将规划数据设置到模块中
  module_ptr->setData(planner_data);
  // 将上一个模块的输出设置到当前模块中
  module_ptr->setPreviousModuleOutput(previous_module_output);

  // 锁定模块的实时控制（RTC）命令，确保线程安全
  module_ptr->lockRTCCommand();
  // 运行模块并获取其输出结果
  const auto result = module_ptr->run();
  // 解锁模块的实时控制命令
  module_ptr->unlockRTCCommand();

  // 执行模块的后处理逻辑
  module_ptr->postProcess();

  // 更新模块的当前状态
  module_ptr->updateCurrentState();

  // 发布感兴趣对象的标记（Marker）
  module_ptr->publishObjectsOfInterestMarker();

  // 记录模块的处理时间，并将结果累加到 processing_time_ 中
  processing_time_.at(module_ptr->name()) += stop_watch.toc(module_ptr->name(), true);

  // 返回模块的输出结果
  return result;
}

SlotOutput SubPlannerManager::runApprovedModules(
  const std::shared_ptr<PlannerData> & data, const BehaviorModuleOutput & upstream_slot_output)
{
  std::unordered_map<std::string, BehaviorModuleOutput> results;
  BehaviorModuleOutput output = upstream_slot_output;
  results.emplace("root", output);

  const bool is_candidate_plan_applied = true /* NOTE: not used in this process */;
  bool is_this_failed = false;
  bool is_this_waiting_approval = false;

  if (approved_module_ptrs_.empty()) {
    return SlotOutput{output, is_candidate_plan_applied, is_this_failed, is_this_waiting_approval};
  }

  // unlock only last approved module
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const auto & m) {
    m->lockOutputPath();
  });
  approved_module_ptrs_.back()->unlockOutputPath();

  /**
   * bootstrap approved module output
   */
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const auto & m) {
    output = run(m, data, output);
    results.emplace(m->name(), output);
  });

  const auto waiting_approval_modules_itr = std::find_if(
    approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
    [](const auto & m) { return m->isWaitingApproval(); });

  if (waiting_approval_modules_itr != approved_module_ptrs_.end()) {
    is_this_waiting_approval = true;

    // only keep this module as candidate
    clearCandidateModules();
    candidate_module_ptrs_.push_back(*waiting_approval_modules_itr);

    // delete following result but keep the rest of the following modules
    std::for_each(
      waiting_approval_modules_itr, approved_module_ptrs_.end(),
      [&results](const auto & m) { results.erase(m->name()); });
    debug_info_.scene_status.emplace_back(
      *waiting_approval_modules_itr, SceneModuleUpdateInfo::Action::MOVE,
      "Back To Waiting Approval");
    approved_module_ptrs_.erase(waiting_approval_modules_itr);

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  /**
   * remove failure modules. these modules' outputs are discarded as invalid plan.
   */
  const auto failed_itr = std::find_if(
    approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
    [](const auto & m) { return m->getCurrentStatus() == ModuleStatus::FAILURE; });
  if (failed_itr != approved_module_ptrs_.end()) {
    is_this_failed = true;

    // clear all candidates
    clearCandidateModules();

    // delete both subsequent result and modules
    std::for_each(failed_itr, approved_module_ptrs_.end(), [&](auto & m) {
      results.erase(m->name());
      debug_info_.scene_status.emplace_back(
        m, SceneModuleUpdateInfo::Action::DELETE, "From Approved");
      deleteExpiredModules(m);
    });
    approved_module_ptrs_.erase(failed_itr, approved_module_ptrs_.end());

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  if (approved_module_ptrs_.empty()) {
    return SlotOutput{
      results.at("root"), is_candidate_plan_applied, is_this_failed, is_this_waiting_approval};
  }

  // use the last module's output as approved modules planning result.
  const auto approved_modules_output = [&results, this]() {
    const auto itr = std::find_if(
      approved_module_ptrs_.rbegin(), approved_module_ptrs_.rend(),
      [&results](const auto & m) { return results.count(m->name()) != 0; });

    if (itr != approved_module_ptrs_.rend()) {
      return results.at((*itr)->name());
    }
    return results.at("root");
  }();

  // if lane change module has succeeded, update current route lanelet.
  if (std::any_of(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [](const auto & m) {
        return m->getCurrentStatus() == ModuleStatus::SUCCESS &&
               m->isCurrentRouteLaneletToBeReset();
      }))
    resetCurrentRouteLanelet(data);

  // remove success module immediately.
  for (auto success_itr = std::find_if(
         approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
         [](const auto & m) { return m->getCurrentStatus() == ModuleStatus::SUCCESS; });
       success_itr != approved_module_ptrs_.end();
       /* success_itr++ */) {
    if ((*success_itr)->getCurrentStatus() == ModuleStatus::SUCCESS) {
      debug_info_.scene_status.emplace_back(
        *success_itr, SceneModuleUpdateInfo::Action::DELETE, "From Approved");
      deleteExpiredModules(*success_itr);
      success_itr = approved_module_ptrs_.erase(success_itr);
    } else {
      success_itr++;
    }
  }

  std::for_each(
    manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });

  return SlotOutput{
    approved_modules_output, is_candidate_plan_applied, is_this_failed, is_this_waiting_approval};
}

// SubPlannerManager 的 propagateFull 函数，用于在多个场景模块之间传播规划结果
SlotOutput SubPlannerManager::propagateFull(
  const std::shared_ptr<PlannerData> & data, const SlotOutput & previous_slot_output)
{
  // 获取当前管理器中模块的数量
  const size_t module_size = manager_ptrs_.size();
  // 计算最大迭代次数：module_size * (module_size + 1) / 2
  const size_t max_iteration_num = static_cast<int>(module_size * (module_size + 1) / 2);

  // 初始化状态标志：是否等待上游模块批准
  bool is_waiting_approved_slot = previous_slot_output.is_upstream_waiting_approved;
  // 初始化状态标志：是否因上游模块失败而无法继续
  bool is_failed_approved_slot = false;
  // 初始化输出路径为上一个槽位的输出路径
  auto output_path = previous_slot_output.valid_output;

  // 用于存储需要删除的模块
  std::vector<SceneModulePtr> deleted_modules;

  // 进入迭代循环，最多执行 max_iteration_num 次
  for (size_t itr_num = 0; itr_num < max_iteration_num; ++itr_num) {

    // 运行所有已批准的模块，获取它们的输出
    const auto approved_module_result = runApprovedModules(data, previous_slot_output.valid_output);
    const auto & approved_module_output = approved_module_result.valid_output;

    // 状态传播：如果任何模块返回“等待上游批准”或“上游失败”，则更新状态标志
    is_waiting_approved_slot =
      is_waiting_approved_slot || approved_module_result.is_upstream_waiting_approved;
    is_failed_approved_slot =
      is_failed_approved_slot || approved_module_result.is_upstream_failed_approved;

    // 获取需要运行的请求模块
    const auto request_modules = getRequestModules(approved_module_output, deleted_modules);

    // 如果没有需要运行的模块，直接返回当前的槽位输出
    if (request_modules.empty()) {
      // there is no module that needs to be launched
      return SlotOutput{
        approved_module_output, isAnyCandidateExclusive(), is_failed_approved_slot,
        is_waiting_approved_slot};
    }

    // 运行请求模块中优先级最高的模块，获取其输出
    const auto [highest_priority_module, candidate_module_output] =
      runRequestModules(request_modules, data, approved_module_output);

    // 如果没有需要运行的模块，直接返回当前的槽位输出
    if (!highest_priority_module) {
      // there is no need to launch new module
      return SlotOutput{
        approved_module_output, isAnyCandidateExclusive(), is_failed_approved_slot,
        is_waiting_approved_slot};
    }

    // 如果最高优先级模块正在等待批准，直接返回当前的槽位输出
    if (highest_priority_module->isWaitingApproval()) {
      // there is no need to launch new module
      return SlotOutput{
        candidate_module_output, isAnyCandidateExclusive(), is_failed_approved_slot,
        is_waiting_approved_slot};
    }

    // 更新输出路径为候选模块的输出
    output_path = candidate_module_output;
    // 将最高优先级模块添加到已批准模块列表
    addApprovedModule(highest_priority_module);
    // 清理候选模块列表
    clearCandidateModules();
  }

  // 在迭代完成后，返回最终的槽位输出
  return SlotOutput{
    output_path, isAnyCandidateExclusive(), is_failed_approved_slot, is_waiting_approved_slot};
}

SlotOutput SubPlannerManager::propagateWithExclusiveCandidate(
  const std::shared_ptr<PlannerData> & data, const SlotOutput & previous_slot_output)
{
  const auto approved_module_result = runApprovedModules(data, previous_slot_output.valid_output);
  const auto & approved_module_output = approved_module_result.valid_output;

  // these status needs to be propagated to downstream slots
  // if any of the slots returned following statuses, keep it
  const bool is_waiting_approved_slot = previous_slot_output.is_upstream_waiting_approved |
                                        approved_module_result.is_upstream_waiting_approved;
  const bool is_failed_approved_slot = previous_slot_output.is_upstream_failed_approved |
                                       approved_module_result.is_upstream_failed_approved;

  // there is no module that needs to be launched
  return SlotOutput{
    approved_module_output, true, is_failed_approved_slot, is_waiting_approved_slot};
}

void SubPlannerManager::propagateWithFailedApproved()
{
  clearCandidateModules();
  clearApprovedModules();
}

SlotOutput SubPlannerManager::propagateWithWaitingApproved(
  const std::shared_ptr<PlannerData> & data, const SlotOutput & previous_slot_output)
{
  clearCandidateModules();

  return previous_slot_output.is_upstream_candidate_exclusive
           ? propagateWithExclusiveCandidate(data, previous_slot_output)
           : propagateFull(data, previous_slot_output);
}

}  // namespace autoware::behavior_path_planner
