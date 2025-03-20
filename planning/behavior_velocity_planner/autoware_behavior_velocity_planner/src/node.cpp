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

#include "autoware/behavior_velocity_planner/node.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_utils/ros/wait_for_param.hpp>
#include <autoware_utils/transform/transforms.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_routing/Route.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <functional>
#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace
{

// 将带有车道ID的路径转换为普通路径
autoware_planning_msgs::msg::Path to_path(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path_with_id)
{
  autoware_planning_msgs::msg::Path path;
  for (const auto & path_point : path_with_id.points) {
    path.points.push_back(path_point.point);
  }
  return path;
}
}  // namespace

// BehaviorVelocityPlannerNode 构造函数
BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("behavior_velocity_planner_node", node_options),
  tf_buffer_(this->get_clock()),  // 初始化 TF 缓冲区
  tf_listener_(tf_buffer_), // 初始化 TF 监听器
  planner_data_(*this)  // 初始化规划器数据
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // 触发订阅器，订阅带有车道ID的路径
  trigger_sub_path_with_lane_id_ =
    this->create_subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>(
      "~/input/path_with_lane_id", 1, std::bind(&BehaviorVelocityPlannerNode::onTrigger, this, _1));

  // 创建加载插件的服务
  srv_load_plugin_ = create_service<autoware_internal_debug_msgs::srv::String>(
    "~/service/load_plugin", std::bind(&BehaviorVelocityPlannerNode::onLoadPlugin, this, _1, _2));

  // 创建卸载插件的服务
  srv_unload_plugin_ = create_service<autoware_internal_debug_msgs::srv::String>(
    "~/service/unload_plugin",
    std::bind(&BehaviorVelocityPlannerNode::onUnloadPlugin, this, _1, _2));

  // 设置速度平滑器参数
  onParam();

  // 创建发布器
  path_pub_ = this->create_publisher<autoware_planning_msgs::msg::Path>("~/output/path", 1);
  debug_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/path", 1);

  // 参数声明
  forward_path_length_ = declare_parameter<double>("forward_path_length");  // 前向路径长度
  backward_path_length_ = declare_parameter<double>("backward_path_length");  // 后向路径长度
  behavior_output_path_interval_ = declare_parameter<double>("behavior_output_path_interval");  // 行为输出路径间隔
  planner_data_.stop_line_extend_length = declare_parameter<double>("stop_line_extend_length"); // 停止线延伸长度

  // 最近搜索参数
  planner_data_.ego_nearest_dist_threshold =
    declare_parameter<double>("ego_nearest_dist_threshold");  // 自车最近距离阈值
  planner_data_.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold"); // 自车最近偏航角阈值

  // 是否为仿真模式
  planner_data_.is_simulation = declare_parameter<bool>("is_simulation");

  // 初始化规划器管理器
  for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
    // 工作区：由于 ROS 2 无法获取空列表，启动器会在参数中设置 ['']。
    if (name == "") {
      break;
    }
    planner_manager_.launchScenePlugin(*this, name);
  }

  // 配置日志级别
  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);

  // 初始化发布时间发布器
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

// 加载插件的回调函数
void BehaviorVelocityPlannerNode::onLoadPlugin(
  const autoware_internal_debug_msgs::srv::String::Request::SharedPtr request,
  [[maybe_unused]] const autoware_internal_debug_msgs::srv::String::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lk(mutex_);
  planner_manager_.launchScenePlugin(*this, request->data); // 启动指定插件
}

// 卸载插件的回调函数
void BehaviorVelocityPlannerNode::onUnloadPlugin(
  const autoware_internal_debug_msgs::srv::String::Request::SharedPtr request,
  [[maybe_unused]] const autoware_internal_debug_msgs::srv::String::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lk(mutex_);
  planner_manager_.removeScenePlugin(*this, request->data); // 移除指定插件
}

// 参数设置回调函数
void BehaviorVelocityPlannerNode::onParam()
{
  // Note(VRichardJP): mutex lock is not necessary as onParam is only called once in the
  // constructed. It would be required if it was a callback. std::lock_guard<std::mutex>
  // lock(mutex_);

  // 初始化速度平滑器
  planner_data_.velocity_smoother_ =
    std::make_unique<autoware::velocity_smoother::AnalyticalJerkConstrainedSmoother>(*this);
  planner_data_.velocity_smoother_->setWheelBase(planner_data_.vehicle_info_.wheel_base_m); // 设置车辆轴距
}

// 处理无地面点云数据
void BehaviorVelocityPlannerNode::processNoGroundPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    // 查找从点云帧到地图帧的变换
    transform = tf_buffer_.lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*msg, pc);  // 将 ROS 消息转换为 PCL 点云

  Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  if (!pc.empty()) {
    autoware_utils::transform_pointcloud(pc, *pc_transformed, affine);  // 变换点云
  }

  planner_data_.no_ground_pointcloud = pc_transformed;  // 存储变换后的点云
}

// 处理里程计数据
void BehaviorVelocityPlannerNode::processOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  auto current_odometry = std::make_shared<geometry_msgs::msg::PoseStamped>();
  current_odometry->header = msg->header;
  current_odometry->pose = msg->pose.pose;
  planner_data_.current_odometry = current_odometry;  // 存储当前里程计数据

  auto current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
  current_velocity->header = msg->header;
  current_velocity->twist = msg->twist.twist;
  planner_data_.current_velocity = current_velocity;  // 存储当前速度数据

  // 将速度数据添加到缓冲区
  planner_data_.velocity_buffer.push_front(*current_velocity);
  const rclcpp::Time now = this->now();
  while (!planner_data_.velocity_buffer.empty()) {
    // 检查最旧数据的时间
    const auto & s = planner_data_.velocity_buffer.back().header.stamp;
    const auto time_diff =
      now >= s ? now - s : rclcpp::Duration(0, 0);  // 注意：负时间会抛出异常。

    // 如果最旧数据的时间小于阈值，则退出循环
    if (time_diff.seconds() <= PlannerData::velocity_buffer_time_sec) {
      break;
    }

    // 移除旧数据
    planner_data_.velocity_buffer.pop_back();
  }
}

// 处理交通信号数据
void BehaviorVelocityPlannerNode::processTrafficSignals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg)
{
  // 清除之前的观测数据
  planner_data_.traffic_light_id_map_raw_.clear();
  const auto traffic_light_id_map_last_observed_old =
    planner_data_.traffic_light_id_map_last_observed_;
  planner_data_.traffic_light_id_map_last_observed_.clear();
  for (const auto & signal : msg->traffic_light_groups) {
    TrafficSignalStamped traffic_signal;
    traffic_signal.stamp = msg->stamp;
    traffic_signal.signal = signal;
    planner_data_.traffic_light_id_map_raw_[signal.traffic_light_group_id] = traffic_signal;
    const bool is_unknown_observation =
      std::any_of(signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      });
    // 如果观测是 UNKNOWN 并且有过去的观测数据，则只更新时间戳并保留信息主体
    const auto old_data =
      traffic_light_id_map_last_observed_old.find(signal.traffic_light_group_id);
    if (is_unknown_observation && old_data != traffic_light_id_map_last_observed_old.end()) {
      // 复制上一次的观测数据
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id] =
        old_data->second;
      // 更新时间戳
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id].stamp =
        msg->stamp;
    } else {
      // 如果 (1) 观测不是 UNKNOWN 或 (2) 第一次观测是 UNKNOWN
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id] =
        traffic_signal;
    }
  }
}

// 检查数据是否准备就绪
bool BehaviorVelocityPlannerNode::processData(rclcpp::Clock clock)
{
  bool is_ready = true;
  const auto & logData = [&clock, this](const std::string & data_type) {
    std::string msg = "Waiting for " + data_type + " data";
    RCLCPP_INFO_THROTTLE(get_logger(), clock, logger_throttle_interval, "%s", msg.c_str());
  };

  const auto & getData = [&logData](auto & dest, auto & sub, const std::string & data_type = "") {
    const auto temp = sub.take_data();
    if (temp) {
      dest = temp;
      return true;
    }
    if (!data_type.empty()) logData(data_type);
    return false;
  };

  is_ready &= getData(planner_data_.current_acceleration, sub_acceleration_, "acceleration");
  is_ready &= getData(planner_data_.predicted_objects, sub_predicted_objects_, "predicted_objects");
  is_ready &= getData(planner_data_.occupancy_grid, sub_occupancy_grid_, "occupancy_grid");

  const auto odometry = sub_vehicle_odometry_.take_data();
  if (odometry) {
    processOdometry(odometry);
  } else {
    logData("odometry");
    is_ready = false;
  }

  const auto no_ground_pointcloud = sub_no_ground_pointcloud_.take_data();
  if (no_ground_pointcloud) {
    processNoGroundPointCloud(no_ground_pointcloud);
  } else {
    logData("pointcloud");
    is_ready = false;
  }

  const auto map_data = sub_lanelet_map_.take_data();
  if (map_data) {
    planner_data_.route_handler_ = std::make_shared<route_handler::RouteHandler>(*map_data);
  }

  // planner_data_.external_velocity_limit is std::optional type variable.
  const auto external_velocity_limit = sub_external_velocity_limit_.take_data();
  if (external_velocity_limit) {
    planner_data_.external_velocity_limit = *external_velocity_limit;
  }

  const auto traffic_signals = sub_traffic_signals_.take_data();
  if (traffic_signals) processTrafficSignals(traffic_signals);

  return is_ready;
}

// 检查数据是否准备就绪
// NOTE: argument planner_data must not be referenced for multithreading
bool BehaviorVelocityPlannerNode::isDataReady(rclcpp::Clock clock)
{
  if (!planner_data_.velocity_smoother_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), clock, logger_throttle_interval,
      "Waiting for the initialization of velocity smoother");
    return false;
  }

  return processData(clock);
}

// 触发回调函数，处理输入路径
void BehaviorVelocityPlannerNode::onTrigger(
  const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg)
{
  std::unique_lock<std::mutex> lk(mutex_);

  if (!isDataReady(*get_clock())) {
    return;
  }

  // 加载地图并检查路由处理器
  if (!planner_data_.route_handler_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), logger_throttle_interval,
      "Waiting for the initialization of route_handler");
    return;
  }

  if (input_path_msg->points.empty()) {
    return;
  }

  // 生成路径
  const autoware_planning_msgs::msg::Path output_path_msg =
    generatePath(input_path_msg, planner_data_);

  lk.unlock();

  // 发布路径
  path_pub_->publish(output_path_msg);
  published_time_publisher_->publish_if_subscribed(path_pub_, output_path_msg.header.stamp);

  if (debug_viz_pub_->get_subscription_count() > 0) {
    publishDebugMarker(output_path_msg);
  }
}

// 生成路径
autoware_planning_msgs::msg::Path BehaviorVelocityPlannerNode::generatePath(
  const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg,
  const PlannerData & planner_data)
{
  autoware_planning_msgs::msg::Path output_path_msg;

  // TODO(someone): 支持反向路径
  const auto is_driving_forward = autoware::motion_utils::isDrivingForward(input_path_msg->points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.value() : is_driving_forward_;
  if (!is_driving_forward_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), logger_throttle_interval,
      "Backward path is NOT supported. just converting path_with_lane_id to path");
    output_path_msg = to_path(*input_path_msg);
    output_path_msg.header.frame_id = "map";
    output_path_msg.header.stamp = this->now();
    output_path_msg.left_bound = input_path_msg->left_bound;
    output_path_msg.right_bound = input_path_msg->right_bound;
    return output_path_msg;
  }

  // 规划路径速度 (核心函数)
  const auto velocity_planned_path = planner_manager_.planPathVelocity(
    std::make_shared<const PlannerData>(planner_data), *input_path_msg);

  // 筛选路径点
  const auto filtered_path =
    autoware::behavior_velocity_planner::filterLitterPathPoint(to_path(velocity_planned_path));

  // 插值路径
  const auto interpolated_path_msg = autoware::behavior_velocity_planner::interpolatePath(
    filtered_path, forward_path_length_, behavior_output_path_interval_);

  // 检查停止点
  output_path_msg = autoware::behavior_velocity_planner::filterStopPathPoint(interpolated_path_msg);

  output_path_msg.header.frame_id = "map";
  output_path_msg.header.stamp = this->now();

  // TODO(someone): 这必须在每个场景模块中更新，但现在从输入消息中复制。
  output_path_msg.left_bound = input_path_msg->left_bound;
  output_path_msg.right_bound = input_path_msg->right_bound;

  return output_path_msg;
}

// 发布调试标记
void BehaviorVelocityPlannerNode::publishDebugMarker(const autoware_planning_msgs::msg::Path & path)
{
  visualization_msgs::msg::MarkerArray output_msg;
  for (size_t i = 0; i < path.points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.pose = path.points.at(i).pose;
    marker.scale.y = marker.scale.z = 0.05;
    marker.scale.x = 0.25;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.color.a = 0.999;  // 不要忘记设置透明度！
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    output_msg.markers.push_back(marker);
  }
  debug_viz_pub_->publish(output_msg);
}
}  // namespace autoware::behavior_velocity_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode)
