// Copyright 2019 Autoware Foundation
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

#include "mission_planner.hpp"

#include "service_utils.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/route_checker.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::mission_planner_universe
{


// 实现路径规划的主逻辑，负责生成和管理车辆的行驶路径。

// 构造函数，初始化节点
MissionPlanner::MissionPlanner(const rclcpp::NodeOptions & options)
: Node("mission_planner", options),
  arrival_checker_(this), // 初始化到达检查器
  plugin_loader_(
    "autoware_mission_planner_universe", "autoware::mission_planner_universe::PlannerPlugin"), // // 插件加载器
  tf_buffer_(get_clock()), // TF 缓冲区
  tf_listener_(tf_buffer_), // // TF 监听器
  odometry_(nullptr), // 初始化里程计为空
  map_ptr_(nullptr) // 初始化地图指针为空
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // 从参数服务器获取地图帧名称
  map_frame_ = declare_parameter<std::string>("map_frame");
  // 获取重新规划路径的时间阈值
  reroute_time_threshold_ = declare_parameter<double>("reroute_time_threshold");
  // 获取重新规划路径的最小长度
  minimum_reroute_length_ = declare_parameter<double>("minimum_reroute_length");
  // ?
  allow_reroute_in_autonomous_mode_ = declare_parameter<bool>("allow_reroute_in_autonomous_mode");

  // 创建默认路径规划器插件实例
  planner_ = plugin_loader_.createSharedInstance(
    "autoware::mission_planner_universe::lanelet2::DefaultPlanner");
  planner_->initialize(this); // 初始化路径规划器

  // QoS（Quality of Service，服务质量）
  // 设置 QoS 为持久化
  const auto durable_qos = rclcpp::QoS(1).transient_local();

  // 订阅里程计信息
  sub_odometry_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS(1), std::bind(&MissionPlanner::on_odometry, this, _1));
  // 订阅操作模式状态
  sub_operation_mode_state_ = create_subscription<OperationModeState>(
    "~/input/operation_mode_state", rclcpp::QoS(1),
    std::bind(&MissionPlanner::on_operation_mode_state, this, _1));
  // 订阅矢量地图
  sub_vector_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", durable_qos, std::bind(&MissionPlanner::on_map, this, _1));
  // 发布路径标记
  pub_marker_ = create_publisher<MarkerArray>("~/debug/route_marker", durable_qos);

  // 注意：路由接口的回调组应该是互斥的
  // NOTE: The route interface should be mutually exclusive by callback group.

  // 订阅修改后的目标点
  sub_modified_goal_ = create_subscription<PoseWithUuidStamped>(
    "~/input/modified_goal", durable_qos, std::bind(&MissionPlanner::on_modified_goal, this, _1));
  // 创建清除路径服务
  srv_clear_route = create_service<ClearRoute>(
    "~/clear_route", service_utils::handle_exception(&MissionPlanner::on_clear_route, this));
  // 创建设置 Lanelet 路径服务
  srv_set_lanelet_route = create_service<SetLaneletRoute>(
    "~/set_lanelet_route",
    service_utils::handle_exception(&MissionPlanner::on_set_lanelet_route, this));
  // 创建设置 Waypoint 路径服务
  srv_set_waypoint_route = create_service<SetWaypointRoute>(
    "~/set_waypoint_route",
    service_utils::handle_exception(&MissionPlanner::on_set_waypoint_route, this));

  // 发布路径
  pub_route_ = create_publisher<LaneletRoute>("~/route", durable_qos);
  // 发布路径状态
  pub_state_ = create_publisher<RouteState>("~/state", durable_qos);

  // Route state will be published when the node gets ready for route api after initialization,
  // otherwise the mission planner rejects the request for the API.
  // 节点初始化后，当准备好路由 API 时，将发布路由状态
  const auto period = rclcpp::Rate(10).period();
  data_check_timer_ = create_wall_timer(period, [this] { check_initialization(); });
  is_mission_planner_ready_ = false;

  // 配置日志记录器
  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
}

// 发布处理时间
void MissionPlanner::publish_processing_time(
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch)
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);
}

// 发布位姿日志
void MissionPlanner::publish_pose_log(const Pose & pose, const std::string & pose_type)
{
  const auto & p = pose.position;
  RCLCPP_INFO(
    this->get_logger(), "%s pose - x: %f, y: %f, z: %f", pose_type.c_str(), p.x, p.y, p.z);
  const auto & quaternion = pose.orientation;
  RCLCPP_INFO(
    this->get_logger(), "%s orientation - qx: %f, qy: %f, qz: %f, qw: %f", pose_type.c_str(),
    quaternion.x, quaternion.y, quaternion.z, quaternion.w);
}

// 检查初始化状态
void MissionPlanner::check_initialization()
{
  auto logger = get_logger();
  auto clock = *get_clock();

  // 如果路径规划器未准备好，等待 Lanelet 地图
  if (!planner_->ready()) {
    RCLCPP_INFO_THROTTLE(logger, clock, 5000, "waiting lanelet map... Route API is not ready.");
    return;
  }
  // 如果里程计数据未收到，等待里程计
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(logger, clock, 5000, "waiting odometry... Route API is not ready.");
    return;
  }

  // 所有数据已准备好，API 可用
  // All data is ready. Now API is available.
  is_mission_planner_ready_ = true;
  RCLCPP_DEBUG(logger, "Route API is ready.");
  change_state(RouteState::UNSET);

  // 停止定时器回调
  // Stop timer callback.
  data_check_timer_->cancel();
  data_check_timer_ = nullptr;
}

// 里程计回调函数
void MissionPlanner::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = msg;

  // 注意：在其他状态下不要检查，因为目标可能会改变
  // NOTE: Do not check in the other states as goal may change.
  if (state_.state == RouteState::SET) {
    PoseStamped pose;
    pose.header = odometry_->header;
    pose.pose = odometry_->pose.pose;
    // 检查是否到达目标点
    if (arrival_checker_.is_arrived(pose)) {
      change_state(RouteState::ARRIVED);
    }
  }
}

// 操作模式状态回调函数
void MissionPlanner::on_operation_mode_state(const OperationModeState::ConstSharedPtr msg)
{
  operation_mode_state_ = msg;
}

// 地图回调函数
void MissionPlanner::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  map_ptr_ = msg;
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_ptr_, lanelet_map_ptr_);
}

// 转换位姿到地图坐标系
Pose MissionPlanner::transform_pose(const Pose & pose, const Header & header)
{
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::Pose result;
  try {
    transform = tf_buffer_.lookupTransform(map_frame_, header.frame_id, tf2::TimePointZero);
    tf2::doTransform(pose, result, transform);
    return result;
  } catch (tf2::TransformException & error) {
    throw service_utils::TransformError(error.what());
  }
}

// 改变路径状态
void MissionPlanner::change_state(RouteState::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

// 修改目标点回调函数
void MissionPlanner::on_modified_goal(const PoseWithUuidStamped::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received modified goal.");

  if (state_.state != RouteState::SET) {
    RCLCPP_ERROR(get_logger(), "The route hasn't set yet. Cannot reroute.");
    return;
  }
  if (!is_mission_planner_ready_) {
    RCLCPP_ERROR(get_logger(), "The mission planner is not ready.");
    return;
  }
  if (!current_route_) {
    RCLCPP_ERROR(get_logger(), "The route has not set yet.");
    return;
  }
  if (current_route_->uuid != msg->uuid) {
    RCLCPP_ERROR(get_logger(), "Goal uuid is incorrect.");
    return;
  }

  change_state(RouteState::REROUTING);
  const auto route = create_route(*msg);

  if (route.segments.empty()) {
    cancel_route();
    change_state(RouteState::SET);
    RCLCPP_ERROR(get_logger(), "The planned route is empty.");
    return;
  }

  change_route(route);
  change_state(RouteState::SET);
  RCLCPP_INFO(get_logger(), "Changed the route with the modified goal");
}

// 清除路径服务回调函数
void MissionPlanner::on_clear_route(
  const ClearRoute::Request::SharedPtr, const ClearRoute::Response::SharedPtr res)
{
  if (!is_mission_planner_ready_) {
    using ResponseCode = autoware_adapi_v1_msgs::msg::ResponseStatus;
    throw service_utils::ServiceException(
      ResponseCode::NO_EFFECT, "The mission planner is not ready.", true);
  }

  change_route();
  change_state(RouteState::UNSET);
  res->status.success = true;
}

// 设置 Lanelet 路径服务回调函数
void MissionPlanner::on_set_lanelet_route(
  const SetLaneletRoute::Request::SharedPtr req, const SetLaneletRoute::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoute::Response;
  const auto is_reroute = state_.state == RouteState::SET;

  if (state_.state != RouteState::UNSET && state_.state != RouteState::SET) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "The route cannot be set in the current state.");
  }
  if (!is_mission_planner_ready_) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The mission planner is not ready.");
  }
  if (is_reroute && !operation_mode_state_) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "Operation mode state is not received.");
  }

  const bool is_autonomous_driving =
    operation_mode_state_ ? operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
                              operation_mode_state_->is_autoware_control_enabled
                          : false;

  if (is_reroute && !allow_reroute_in_autonomous_mode_ && is_autonomous_driving) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "Reroute is not allowed in autonomous mode.");
  }

  if (is_reroute && is_autonomous_driving) {
    const auto reroute_availability = sub_reroute_availability_.take_data();
    if (!reroute_availability || !reroute_availability->availability) {
      throw service_utils::ServiceException(
        ResponseCode::ERROR_INVALID_STATE,
        "Cannot reroute as the planner is not in lane following.");
    }
  }

  change_state(is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  const auto route = create_route(*req);

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
  }

  if (is_reroute && is_autonomous_driving && !check_reroute_safety(*current_route_, route)) {
    cancel_route();
    change_state(RouteState::SET);
    throw service_utils::ServiceException(
      ResponseCode::ERROR_REROUTE_FAILED, "New route is not safe. Reroute failed.");
  }

  change_route(route);
  change_state(RouteState::SET);
  res->status.success = true;

  publish_pose_log(odometry_->pose.pose, "initial");
  publish_pose_log(req->goal_pose, "goal");
}

// 设置 Waypoint 路径服务回调函数
void MissionPlanner::on_set_waypoint_route(
  const SetWaypointRoute::Request::SharedPtr req, const SetWaypointRoute::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoutePoints::Response;
  const auto is_reroute = state_.state == RouteState::SET;

  if (state_.state != RouteState::UNSET && state_.state != RouteState::SET) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "The route cannot be set in the current state.");
  }
  if (!is_mission_planner_ready_) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The mission planner is not ready.");
  }
  if (is_reroute && !operation_mode_state_) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "Operation mode state is not received.");
  }

  const bool is_autonomous_driving =
    operation_mode_state_ ? operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
                              operation_mode_state_->is_autoware_control_enabled
                          : false;

  if (is_reroute && is_autonomous_driving) {
    const auto reroute_availability = sub_reroute_availability_.take_data();
    if (!reroute_availability || !reroute_availability->availability) {
      throw service_utils::ServiceException(
        ResponseCode::ERROR_INVALID_STATE,
        "Cannot reroute as the planner is not in lane following.");
    }
  }

  change_state(is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  const auto route = create_route(*req);

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
  }

  if (is_reroute && is_autonomous_driving && !check_reroute_safety(*current_route_, route)) {
    cancel_route();
    change_state(RouteState::SET);
    throw service_utils::ServiceException(
      ResponseCode::ERROR_REROUTE_FAILED, "New route is not safe. Reroute failed.");
  }

  change_route(route);
  change_state(RouteState::SET);
  res->status.success = true;

  publish_pose_log(odometry_->pose.pose, "initial");
  publish_pose_log(req->goal_pose, "goal");
}

// 改变路径
void MissionPlanner::change_route()
{
  current_route_ = nullptr;
  planner_->clearRoute();
  arrival_checker_.set_goal();

  // TODO(Takagi, Isamu): publish an empty route here
  // pub_route_->publish();
  // pub_marker_->publish();
}


// 改变路径
void MissionPlanner::change_route(const LaneletRoute & route)
{
  PoseWithUuidStamped goal;
  goal.header = route.header;
  goal.pose = route.goal_pose;
  goal.uuid = route.uuid;

  current_route_ = std::make_shared<LaneletRoute>(route);
  planner_->updateRoute(route);
  arrival_checker_.set_goal(goal);

  pub_route_->publish(route);
  pub_marker_->publish(planner_->visualize(route));
}

// 取消路径
void MissionPlanner::cancel_route()
{
  // 恢复路径规划器状态
  // Restore planner state that changes with create_route function.
  if (current_route_) {
    planner_->updateRoute(*current_route_);
  }
}

// 创建 Lanelet 路径
LaneletRoute MissionPlanner::create_route(const SetLaneletRoute::Request & req)
{
  const auto & header = req.header;
  const auto & segments = req.segments;
  const auto & goal_pose = req.goal_pose;
  const auto & uuid = req.uuid;
  const auto & allow_goal_modification = req.allow_modification;

  return create_route(header, segments, goal_pose, uuid, allow_goal_modification);
}

// 创建 Waypoint 路径
LaneletRoute MissionPlanner::create_route(const SetWaypointRoute::Request & req)
{
  const auto & header = req.header;
  const auto & waypoints = req.waypoints;
  const auto & goal_pose = req.goal_pose;
  const auto & uuid = req.uuid;
  const auto & allow_goal_modification = req.allow_modification;

  return create_route(
    header, waypoints, odometry_->pose.pose, goal_pose, uuid, allow_goal_modification);
}

// 创建路径
LaneletRoute MissionPlanner::create_route(const PoseWithUuidStamped & msg)
{
  const auto & header = msg.header;
  const auto & goal_pose = msg.pose;
  const auto & uuid = msg.uuid;
  const auto & allow_goal_modification = current_route_->allow_modification;

  // NOTE: Reroute by modifed goal is assumed to be a slight modification near the goal lane.
  //       It is assumed that ego and goal are on the extension of the current route at least.
  //       Therefore, the start pose is the start pose of the current route if it exists.
  //       This prevents the route from becoming shorter due to reroute.
  //       Also, use start pose and waypoints that are on the preferred lanelet of the current route
  //       as much as possible.
  //       For this process, refer to RouteHandler::planPathLaneletsBetweenCheckpoints() or
  //       https://github.com/autowarefoundation/autoware.universe/pull/8238 too.
  const auto & start_pose = current_route_ ? current_route_->start_pose : odometry_->pose.pose;
  std::vector<Pose> waypoints{};
  if (current_route_) {
    waypoints.push_back(odometry_->pose.pose);
  }

  return create_route(header, waypoints, start_pose, goal_pose, uuid, allow_goal_modification);
}

// 创建 Lanelet 路径
LaneletRoute MissionPlanner::create_route(
  const Header & header, const std::vector<LaneletSegment> & segments, const Pose & goal_pose,
  const UUID & uuid, const bool allow_goal_modification)
{
  LaneletRoute route;
  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;
  route.start_pose = odometry_->pose.pose;
  route.goal_pose = transform_pose(goal_pose, header);
  route.segments = segments;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;
  return route;
}

// 创建 Waypoint 路径
LaneletRoute MissionPlanner::create_route(
  const Header & header, const std::vector<Pose> & waypoints, const Pose & start_pose,
  const Pose & goal_pose, const UUID & uuid, const bool allow_goal_modification)
{
  PlannerPlugin::RoutePoints points;
  points.push_back(start_pose);
  for (const auto & waypoint : waypoints) {
    points.push_back(transform_pose(waypoint, header));
  }
  points.push_back(transform_pose(goal_pose, header));

  LaneletRoute route = planner_->plan(points);
  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;
  return route;
}

// 检查重新规划路径的安全性
// 提取前后路径的公共部分，如果长度大于阈值，认为是安全的
bool MissionPlanner::check_reroute_safety(
  const LaneletRoute & original_route, const LaneletRoute & target_route)
{

  // 安全检查
  if (
    original_route.segments.empty() || target_route.segments.empty() || !map_ptr_ ||
    !lanelet_map_ptr_ || !odometry_) {
    RCLCPP_ERROR(get_logger(), "Check reroute safety failed. Route, map or odometry is not set.");
    return false;
  }

  const auto current_velocity = odometry_->twist.twist.linear.x;

  // 如果车辆停止，不检查安全性
  // if vehicle is stopped, do not check safety
  if (current_velocity < 0.01) {
    return true;
  }

  auto hasSamePrimitives = [](
                             const std::vector<LaneletPrimitive> & original_primitives,
                             const std::vector<LaneletPrimitive> & target_primitives) {
    if (original_primitives.size() != target_primitives.size()) {
      return false;
    }

    for (const auto & primitive : original_primitives) {
      const auto has_same = [&](const auto & p) { return p.id == primitive.id; };
      const bool is_same =
        std::find_if(target_primitives.begin(), target_primitives.end(), has_same) !=
        target_primitives.end();
      if (!is_same) {
        return false;
      }
    }
    return true;
  };

  // =============================================================================================
  // NOTE: the target route is calculated while ego is driving on the original route, so basically
  // the first lane of the target route should be in the original route lanelets. So the common
  // segment interval matches the beginning of the target route. The exception is that if ego is
  // on an intersection lanelet, getClosestLanelet() may not return the same lanelet which exists
  // in the original route. In that case the common segment interval does not match the beginning of
  // the target lanelet
  // =============================================================================================

  // =============================================================================================
  // 注意：目标路径是在车辆在原始路径上行驶时计算的，因此目标路径的第一条车道应在原始路径的车道中。
  // 所以共同段间隔匹配目标路径的开始。例外情况是如果车辆在交叉口车道上，getClosestLanelet() 可能不会返回原始路径中存在的车道。
  // 在这种情况下，共同段间隔不匹配目标车道的开始。
  // =============================================================================================

  // 判断两条路由是否交叉部分，设置为start_idx_opt
  const auto start_idx_opt =
    std::invoke([&]() -> std::optional<std::pair<size_t /* original */, size_t /* target */>> {
      for (size_t i = 0; i < original_route.segments.size(); ++i) {
        const auto & original_segment = original_route.segments.at(i).primitives;
        for (size_t j = 0; j < target_route.segments.size(); ++j) {
          const auto & target_segment = target_route.segments.at(j).primitives;
          if (hasSamePrimitives(original_segment, target_segment)) {
            return std::make_pair(i, j);
          }
        }
      }
      return std::nullopt;
    });

  // 如果没找到，直接认为不安全
  if (!start_idx_opt.has_value()) {
    RCLCPP_ERROR(
      get_logger(), "Check reroute safety failed. Cannot find the start index of the route.");
    return false;
  }
  const auto [start_idx_original, start_idx_target] = start_idx_opt.value();

  // 找到与目标原语匹配的最后一个索引
  // find last idx that matches the target primitives
  size_t end_idx_original = start_idx_original;
  size_t end_idx_target = start_idx_target;
  for (size_t i = 1; i < target_route.segments.size() - start_idx_target; ++i) {
    if (start_idx_original + i > original_route.segments.size() - 1) {
      break;
    }

    const auto & original_primitives =
      original_route.segments.at(start_idx_original + i).primitives;
    const auto & target_primitives = target_route.segments.at(start_idx_target + i).primitives;
    if (!hasSamePrimitives(original_primitives, target_primitives)) {
      break;
    }
    end_idx_original = start_idx_original + i;
    end_idx_target = start_idx_target + i;
  }

  // 在从主路径/MRM 到 MRM/主路径的第一次转换时，route_selector 请求的路径可能不是从当前车道开始的
  // at the very first transition from main/MRM to MRM/main, the requested route from the
  // route_selector may not begin from ego current lane (because route_selector requests the
  // previous route once, and then replan)

  // 判断车辆是否在target_route的某个segment中
  const bool ego_is_on_first_target_section = std::any_of(
    target_route.segments.front().primitives.begin(),
    target_route.segments.front().primitives.end(), [&](const auto & primitive) {
      const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
      return lanelet::utils::isInLanelet(target_route.start_pose, lanelet);
    });

  // 如果没找到，直接报错
  if (!ego_is_on_first_target_section) {
    RCLCPP_ERROR(
      get_logger(),
      "Check reroute safety failed. Ego is not on the first section of target route.");
    return false;
  }

  // 如果目标路径的前面不是共同段的前面，则预计目标路径的前面与另一条车道冲突
  // if the front of target route is not the front of common segment, it is expected that the front
  // of the target route is conflicting with another lane which is equal to original
  // route[start_idx_original-1]
  double accumulated_length = 0.0;

  if (start_idx_target != 0 && start_idx_original > 1) {
    // 计算从当前位置到共同段开始的距离
    // compute distance from the current pose to the beginning of the common segment
    const auto current_pose = target_route.start_pose;
    const auto primitives = original_route.segments.at(start_idx_original - 1).primitives;

    // 提取起始primitives的所有lanelet
    lanelet::ConstLanelets start_lanelets;
    for (const auto & primitive : primitives) {
      const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
      start_lanelets.push_back(lanelet);
    }

    // 从候选lanelet获取最近的lanelet，赋值到closest_lanelet
    // closest lanelet in start lanelets
    lanelet::ConstLanelet closest_lanelet;
    if (!lanelet::utils::query::getClosestLanelet(start_lanelets, current_pose, &closest_lanelet)) {
      RCLCPP_ERROR(get_logger(), "Check reroute safety failed. Cannot find the closest lanelet.");
      return false;
    }

    const auto & centerline_2d = lanelet::utils::to2D(closest_lanelet.centerline());
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(current_pose.position);
    const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
      centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
    const double dist_to_current_pose = arc_coordinates.length;
    const double lanelet_length = lanelet::utils::getLaneletLength2d(closest_lanelet);
    accumulated_length = lanelet_length - dist_to_current_pose;
  } else {
    // 计算从当前位置到当前车道末端的距离
    // compute distance from the current pose to the end of the current lanelet
    const auto current_pose = target_route.start_pose;
    const auto primitives = original_route.segments.at(start_idx_original).primitives;
    lanelet::ConstLanelets start_lanelets;
    for (const auto & primitive : primitives) {
      const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
      start_lanelets.push_back(lanelet);
    }
    // closest lanelet in start lanelets
    lanelet::ConstLanelet closest_lanelet;
    if (!lanelet::utils::query::getClosestLanelet(start_lanelets, current_pose, &closest_lanelet)) {
      RCLCPP_ERROR(get_logger(), "Check reroute safety failed. Cannot find the closest lanelet.");
      return false;
    }

    const auto & centerline_2d = lanelet::utils::to2D(closest_lanelet.centerline());
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(current_pose.position);
    const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
      centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
    const double dist_to_current_pose = arc_coordinates.length;
    const double lanelet_length = lanelet::utils::getLaneletLength2d(closest_lanelet);
    accumulated_length = lanelet_length - dist_to_current_pose;
  }

  // compute distance from the start_idx+1 to end_idx
  for (size_t i = start_idx_original + 1; i <= end_idx_original; ++i) {
    const auto primitives = original_route.segments.at(i).primitives;
    if (primitives.empty()) {
      break;
    }

    std::vector<double> lanelets_length(primitives.size());
    for (size_t primitive_idx = 0; primitive_idx < primitives.size(); ++primitive_idx) {
      const auto & primitive = primitives.at(primitive_idx);
      const auto & lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
      lanelets_length.at(primitive_idx) = (lanelet::utils::getLaneletLength2d(lanelet));
    }
    accumulated_length += *std::min_element(lanelets_length.begin(), lanelets_length.end());
  }

  // 减去目标点到多余的距离
  // check if the goal is inside of the target terminal lanelet
  const auto & target_end_primitives = target_route.segments.at(end_idx_target).primitives;
  const auto & target_goal = target_route.goal_pose;
  for (const auto & target_end_primitive : target_end_primitives) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(target_end_primitive.id);
    if (lanelet::utils::isInLanelet(target_goal, lanelet)) {
      const auto target_goal_position =
        lanelet::utils::conversion::toLaneletPoint(target_goal.position);
      const double dist_to_goal = lanelet::geometry::toArcCoordinates(
                                    lanelet::utils::to2D(lanelet.centerline()),
                                    lanelet::utils::to2D(target_goal_position).basicPoint())
                                    .length;
      const double target_lanelet_length = lanelet::utils::getLaneletLength2d(lanelet);
      // NOTE: `accumulated_length` here contains the length of the entire target_end_primitive, so
      // the remaining distance from the goal to the end of the target_end_primitive needs to be
      // subtracted.
      const double remaining_dist = target_lanelet_length - dist_to_goal;
      accumulated_length = std::max(accumulated_length - remaining_dist, 0.0);
      break;
    }
  }

  // 如果公共部分的距离大于给定值，认为路径安全
  // check safety
  const double safety_length =
    std::max(current_velocity * reroute_time_threshold_, minimum_reroute_length_);
  if (accumulated_length > safety_length) {
    return true;
  }

  RCLCPP_WARN(
    get_logger(),
    "Length of lane where original and B target (= %f) is less than safety length (= %f), so "
    "reroute is not safe.",
    accumulated_length, safety_length);
  return false;
}
}  // namespace autoware::mission_planner_universe

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_planner_universe::MissionPlanner)
