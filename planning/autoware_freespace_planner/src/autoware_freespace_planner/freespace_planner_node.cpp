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
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoware/freespace_planner/freespace_planner_node.hpp"

#include "autoware/freespace_planner/utils.hpp"
#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/system/stop_watch.hpp>

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::freespace_planner
{

// 自由空间规划节点类
FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options)
{
  using std::placeholders::_1;

  // 初始化节点参数
  {
    auto & p = node_param_;
    p.planning_algorithm = declare_parameter<std::string>("planning_algorithm");  // 规划算法名称
    p.waypoints_velocity = declare_parameter<double>("waypoints_velocity"); // 路点速度
    p.update_rate = declare_parameter<double>("update_rate"); // 更新频率
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m"); // 到达目标的距离阈值
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec"); // 停止时间阈值
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps"); // 停止速度阈值
    p.th_course_out_distance_m = declare_parameter<double>("th_course_out_distance_m"); // 路线偏离距离阈值
    p.th_obstacle_time_sec = declare_parameter<double>("th_obstacle_time_sec");   // 障碍物检测时间阈值
    p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m"); // 车辆形状扩展边距
    p.replan_when_obstacle_found = declare_parameter<bool>("replan_when_obstacle_found"); // 发现障碍物时是否重新规划
    p.replan_when_course_out = declare_parameter<bool>("replan_when_course_out"); // 偏离路线时是否重新规划
  }

  // 设置车辆信息
  {
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo(); // 获取车辆信息
    vehicle_shape_.length = vehicle_info.vehicle_length_m;  // 车辆长度
    vehicle_shape_.width = vehicle_info.vehicle_width_m;    // 车辆宽度
    vehicle_shape_.base_length = vehicle_info.wheel_base_m; // 车辆轴距
    vehicle_shape_.max_steering = vehicle_info.max_steer_angle_rad; // 最大转向角
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;  // 车辆后悬
  }

  // 初始化规划算法
  initializePlanningAlgorithm();

  // 创建订阅者
  route_sub_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&FreespacePlannerNode::onRoute, this, _1));

  // 创建发布者
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // 设置为持久化
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos); // 发布轨迹
    debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", qos); // 发布调试姿态数组
    debug_partial_pose_array_pub_ = create_publisher<PoseArray>("~/debug/partial_pose_array", qos); // 发布部分调试姿态数组
    parking_state_pub_ = create_publisher<std_msgs::msg::Bool>("is_completed", qos);  // 发布停车状态
    processing_time_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", 1);
  }

  // 初始化TF缓冲区和监听器
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // 创建定时器
  {
    const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();  // 计算更新周期
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&FreespacePlannerNode::onTimer, this)); // 创建定时器
  }

  // 初始化日志配置
  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
}

// 获取规划通用参数
PlannerCommonParam FreespacePlannerNode::getPlannerCommonParam()
{
  PlannerCommonParam p;

  // 搜索配置
  p.time_limit = declare_parameter<double>("time_limit"); // 时间限制

  p.theta_size = declare_parameter<int>("theta_size");  // 角度分辨率
  p.angle_goal_range = declare_parameter<double>("angle_goal_range"); // 目标角度范围
  p.curve_weight = declare_parameter<double>("curve_weight"); // 曲线权重
  p.reverse_weight = declare_parameter<double>("reverse_weight"); // 倒车权重
  p.direction_change_weight = declare_parameter<double>("direction_change_weight"); // 方向变化权重
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range"); // 横向目标范围
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range"); // 纵向目标范围
  p.max_turning_ratio = declare_parameter<double>("max_turning_ratio"); // 最大转向比
  p.turning_steps = declare_parameter<int>("turning_steps");  // 转向步数

  // 成本图配置
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");  // 障碍物阈值

  return p;
}

// 判断是否需要重新规划
bool FreespacePlannerNode::isPlanRequired()
{
  if (trajectory_.points.empty()) {
    return true;  // 如果轨迹为空，则需要重新规划
  }

  if (node_param_.replan_when_obstacle_found && checkCurrentTrajectoryCollision()) {
    RCLCPP_DEBUG(get_logger(), "Found obstacle");
    return true;  // 如果发现障碍物且需要重新规划，则返回true
  }

  if (node_param_.replan_when_course_out) {
    const bool is_course_out = utils::calc_distance_2d(trajectory_, current_pose_.pose) >
                               node_param_.th_course_out_distance_m;
    if (is_course_out) {
      RCLCPP_INFO(get_logger(), "Course out");
      return true;  // 如果偏离路线且需要重新规划，则返回true
    }
  }

  return false;
}

// 检查当前轨迹是否与障碍物碰撞
bool FreespacePlannerNode::checkCurrentTrajectoryCollision()
{
  algo_->setMap(*occupancy_grid_);  // 设置地图

  const size_t nearest_index_partial = autoware::motion_utils::findNearestIndex(
    partial_trajectory_.points, current_pose_.pose.position); // 查找最近点索引
  const size_t end_index_partial = partial_trajectory_.points.size() - 1;
  const auto forward_trajectory = utils::get_partial_trajectory(
    partial_trajectory_, nearest_index_partial, end_index_partial, get_clock());  // 获取前向轨迹

  const bool is_obs_found =
    algo_->hasObstacleOnTrajectory(utils::trajectory_to_pose_array(forward_trajectory));  // 检查轨迹上是否有障碍物

  if (!is_obs_found) {
    obs_found_time_ = {}; // 如果没有发现障碍物，则重置时间
    return false;
  }

  if (!obs_found_time_) obs_found_time_ = get_clock()->now(); // 如果是第一次发现障碍物，则记录时间

  return (get_clock()->now() - obs_found_time_.get()).seconds() > node_param_.th_obstacle_time_sec; // 判断是否超过障碍物检测时间阈值
}

// 更新目标索引
void FreespacePlannerNode::updateTargetIndex()
{
  if (!utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps)) {
    return; // 如果车辆未停止，则不更新目标索引
  }

  const auto is_near_target = utils::is_near_target(
    trajectory_.points.at(target_index_).pose, current_pose_.pose,
    node_param_.th_arrived_distance_m); // 判断是否到达目标点

  if (!is_near_target) return;

  const auto new_target_index =
    utils::get_next_target_index(trajectory_.points.size(), reversing_indices_, target_index_); // 获取下一个目标索引

  if (new_target_index == target_index_) {
    // 如果已经完成所有部分轨迹的发布
    is_completed_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Freespace planning completed");
    std_msgs::msg::Bool is_completed_msg;
    is_completed_msg.data = is_completed_;
    parking_state_pub_->publish(is_completed_msg);  // 发布停车状态
  } else {
    // 切换到下一个部分轨迹
    prev_target_index_ = target_index_;
    target_index_ = new_target_index;
  }
}

// 处理路线信息
void FreespacePlannerNode::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  route_ = msg; // 更新路线信息

  goal_pose_.header = msg->header;
  goal_pose_.pose = msg->goal_pose; // 更新目标点

  is_new_parking_cycle_ = true; // 标记为新的停车周期

  reset();  // 重置规划状态
}

// 处理里程计信息
void FreespacePlannerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;  // 更新里程计信息

  odom_buffer_.push_back(msg);  // 将里程计信息加入缓冲区

  // 删除缓冲区中过时的数据
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);

    if (time_diff.seconds() < node_param_.th_stopped_time_sec) {
      break;
    }

    odom_buffer_.pop_front();
  }
}

// 更新数据
void FreespacePlannerNode::updateData()
{
  occupancy_grid_ = occupancy_grid_sub_.take_data();  // 更新占用网格信息

  {
    auto msgs = odom_sub_.take_data();  // 更新里程计信息
    for (const auto & msg : msgs) {
      onOdometry(msg);
    }
  }
}

// 判断数据是否准备就绪
bool FreespacePlannerNode::isDataReady()
{
  bool is_ready = true;

  if (!route_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for route data.");
    is_ready = false; // 如果没有路线信息，则数据未准备就绪
  }

  if (!occupancy_grid_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for occupancy grid.");
    is_ready = false; // 如果没有占用网格信息，则数据未准备就绪
  }

  if (!odom_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry.");
    is_ready = false; // 如果没有里程计信息，则数据未准备就绪
  }

  return is_ready;
}

// 定时器回调函数
void FreespacePlannerNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  scenario_ = scenario_sub_.take_data();  // 获取场景信息
  if (!utils::is_active(scenario_)) { // 如果场景不活跃，则重置规划状态
    reset();
    return;
  }

  updateData(); // 更新数据

  if (!isDataReady()) {
    return; // 如果数据未准备就绪，则直接返回
  }

  if (is_completed_) {
    partial_trajectory_.header = odom_->header;
    const auto stop_trajectory = utils::create_stop_trajectory(partial_trajectory_);  // 创建停止轨迹
    trajectory_pub_->publish(stop_trajectory);  // 发布停止轨迹
    return;
  }

  // 获取当前姿态
  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;

  if (current_pose_.header.frame_id == "") {
    return; // 如果当前姿态的参考系为空，则直接返回
  }

  // 必须在重新规划任何新轨迹之前停车
  const bool is_reset_required = !reset_in_progress_ && isPlanRequired();
  if (is_reset_required) {
    // Stop before planning new trajectory, except in a new parking cycle as the vehicle already
    // stops.
    if (!is_new_parking_cycle_) {
      const auto stop_trajectory = partial_trajectory_.points.empty()
                                     ? utils::create_stop_trajectory(current_pose_, get_clock())
                                     : utils::create_stop_trajectory(partial_trajectory_);
      trajectory_pub_->publish(stop_trajectory);  // 发布停止轨迹
      debug_pose_array_pub_->publish(utils::trajectory_to_pose_array(stop_trajectory)); // 发布调试姿态数组
      debug_partial_pose_array_pub_->publish(utils::trajectory_to_pose_array(stop_trajectory)); // 发布部分调试姿态数组
    }

    reset();  // 重置规划状态

    reset_in_progress_ = true;
  }

  if (reset_in_progress_) {
    const auto is_ego_stopped =
      utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps); // 判断车辆是否停止
    if (is_ego_stopped) {
      // 规划新轨迹
      planTrajectory();
      reset_in_progress_ = false;
    } else {
      // 保持当前停止轨迹
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for the vehicle to stop before generating a new trajectory.");
    }
  }

  // 停止轨迹
  if (trajectory_.points.size() <= 1) {
    is_new_parking_cycle_ = false;
    return;
  }

  // 更新部分轨迹
  updateTargetIndex();
  partial_trajectory_ =
    utils::get_partial_trajectory(trajectory_, prev_target_index_, target_index_, get_clock()); // 获取部分轨迹

  // 发布消息
  trajectory_pub_->publish(partial_trajectory_);  // 发布轨迹
  debug_pose_array_pub_->publish(utils::trajectory_to_pose_array(trajectory_)); // 发布调试姿态数组
  debug_partial_pose_array_pub_->publish(utils::trajectory_to_pose_array(partial_trajectory_)); // 发布部分调试姿态数组

  is_new_parking_cycle_ = false;

  // Publish ProcessingTime
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_->publish(processing_time_msg);
}

// 规划轨迹
void FreespacePlannerNode::planTrajectory()
{
  if (occupancy_grid_ == nullptr) {
    return; // 如果占用网格为空，则直接返回
  }

  // 为规划器提供地图和车辆形状
  algo_->setMap(*occupancy_grid_);

  // 计算成本图坐标系中的姿态
  const auto current_pose_in_costmap_frame = utils::transform_pose(
    current_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = utils::transform_pose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // 执行规划
  const rclcpp::Time start = get_clock()->now();
  std::string error_msg;
  bool result = false;
  try {
    result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);  // 调用规划算法
  } catch (const std::exception & e) {
    error_msg = e.what(); // 捕获异常信息
  }
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_DEBUG(get_logger(), "Freespace planning: %f [s]", (end - start).seconds());  // 输出规划时间

  if (result) {
    RCLCPP_DEBUG(get_logger(), "Found goal!");
    trajectory_ = utils::create_trajectory(
      current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);  // 创建轨迹
    reversing_indices_ = utils::get_reversing_indices(trajectory_); // 获取倒车索引
    prev_target_index_ = 0;
    target_index_ = utils::get_next_target_index(
      trajectory_.points.size(), reversing_indices_, prev_target_index_); // 获取目标索引
  } else {
    RCLCPP_INFO(get_logger(), "Can't find goal: %s", error_msg.c_str());  // 输出规划失败信息
    reset();  // 重置规划状态
  }
}

// 重置规划状态
void FreespacePlannerNode::reset()
{
  trajectory_ = Trajectory(); // 清空轨迹
  partial_trajectory_ = Trajectory(); // 清空部分轨迹
  is_completed_ = false;  // 重置完成标志
  std_msgs::msg::Bool is_completed_msg;
  is_completed_msg.data = is_completed_;
  parking_state_pub_->publish(is_completed_msg);  // 发布停车状态
  obs_found_time_ = {}; // 重置障碍物发现时间
}

// 获取坐标变换
TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));  // 查询坐标变换
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());  // 捕获异常并输出错误信息
  }
  return tf;
}

// 初始化规划算法
void FreespacePlannerNode::initializePlanningAlgorithm()
{
  // 扩展车辆形状
  autoware::freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;
  extended_vehicle_shape.setMinMaxDimension();  // 设置车辆形状的最小和最大尺寸

  const auto planner_common_param = getPlannerCommonParam();  // 获取规划通用参数

  const auto algo_name = node_param_.planning_algorithm;  // 获取规划算法名称

  // 初始化指定的规划算法
  if (algo_name == "astar") {
    algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, *this); // 初始化A*搜索算法
  } else if (algo_name == "rrtstar") {
    algo_ = std::make_unique<RRTStar>(planner_common_param, extended_vehicle_shape, *this); // 初始化RRT*算法
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");  // 抛出异常
  }
  RCLCPP_INFO_STREAM(get_logger(), "initialize planning algorithm: " << algo_name); // 输出初始化信息
}
}  // namespace autoware::freespace_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::freespace_planner::FreespacePlannerNode)
