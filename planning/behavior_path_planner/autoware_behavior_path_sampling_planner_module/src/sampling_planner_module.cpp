// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_sampling_planner_module/sampling_planner_module.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

// 使用命名空间中的工具函数
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::findNearestIndex;
using autoware::motion_utils::findNearestSegmentIndex;
using autoware_utils::calc_distance2d;
using autoware_utils::calc_offset_pose;
using autoware_utils::get_point;
using autoware_utils::Point2d;
using geometry_msgs::msg::Point;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// 采样规划器模块的构造函数
SamplingPlannerModule::SamplingPlannerModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<SamplingPlannerParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface},  // NOLINT
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo()}
{
  // 初始化内部参数
  internal_params_ = std::make_shared<SamplingPlannerInternalParameters>();
  updateModuleParams(parameters);

  // 添加硬约束：路径不能为空
  hard_constraints_.emplace_back(
    [](
      autoware::sampler_common::Path & path,
      [[maybe_unused]] const autoware::sampler_common::Constraints & constraints,
      [[maybe_unused]] const MultiPoint2d & footprint) -> bool {
      return !path.points.empty() && !path.poses.empty();
    });

  // 添加硬约束：路径必须在可行驶区域内且无碰撞
  hard_constraints_.emplace_back(
    [](
      autoware::sampler_common::Path & path,
      const autoware::sampler_common::Constraints & constraints,
      const MultiPoint2d & footprint) -> bool {
      if (!footprint.empty()) {
        path.constraint_results.inside_drivable_area =
          bg::within(footprint, constraints.drivable_polygons);
      }

      for (const auto & f : footprint) {
        const auto collision_index = constraints.rtree.qbegin(bgi::intersects(f));
        if (collision_index != constraints.rtree.qend()) {
          path.constraint_results.collision_free = false;
          break;
        }
      }

      return path.constraint_results.collision_free && path.constraint_results.inside_drivable_area;
    });

  // 添加硬约束：路径的曲率必须在允许范围内
  hard_constraints_.emplace_back(
    [](
      autoware::sampler_common::Path & path,
      const autoware::sampler_common::Constraints & constraints,
      [[maybe_unused]] const MultiPoint2d & footprint) -> bool {
      if (path.curvatures.empty()) {
        path.constraint_results.valid_curvature = false;
        return false;
      }
      const bool curvatures_satisfied =
        std::all_of(path.curvatures.begin(), path.curvatures.end(), [&](const auto & v) -> bool {
          return (v > constraints.hard.min_curvature) && (v < constraints.hard.max_curvature);
        });
      path.constraint_results.valid_curvature = curvatures_satisfied;
      return curvatures_satisfied;
    });

  // TODO(Daniel): 可能需要添加一些软约束或优化目标
  // 例如：路径与中心线的平均距离、防止路径抖动的机制等

  // 添加软约束：路径剩余长度
  soft_constraints_.emplace_back(
    [&](
      autoware::sampler_common::Path & path,
      [[maybe_unused]] const autoware::sampler_common::Constraints & constraints,
      [[maybe_unused]] const SoftConstraintsInputs & input_data) -> double {
      if (path.points.empty()) return 0.0;
      if (path.poses.empty()) return 0.0;

      const auto & ego_pose = input_data.ego_pose;

      const double max_target_length = *std::max_element(
        internal_params_->sampling.target_lengths.begin(),
        internal_params_->sampling.target_lengths.end());

      const auto closest_path_pose_itr = std::min_element(
        path.poses.begin(), path.poses.end(), [&ego_pose](const auto & p1, const auto & p2) {
          const auto dist1 =
            std::hypot(p1.position.x - ego_pose.position.x, p1.position.y - ego_pose.position.y);
          const auto dist2 =
            std::hypot(p2.position.x - ego_pose.position.x, p2.position.y - ego_pose.position.y);
          return dist1 < dist2;
        });

      int closest_path_pose_index = static_cast<int>(closest_path_pose_itr - path.poses.begin());

      const double remaining_path_length =
        path.lengths.back() - path.lengths.at(closest_path_pose_index);

      constexpr double epsilon = 1E-5;
      if (remaining_path_length < epsilon) return max_target_length;
      return max_target_length / remaining_path_length;
    });

  // 添加软约束：路径与中心线的横向距离
  soft_constraints_.emplace_back(
    [&](
      [[maybe_unused]] autoware::sampler_common::Path & path,
      [[maybe_unused]] const autoware::sampler_common::Constraints & constraints,
      [[maybe_unused]] const SoftConstraintsInputs & input_data) -> double {
      if (path.poses.empty()) return 0.0;
      const auto & last_pose = path.poses.back();
      const auto path_point_arc =
        lanelet::utils::getArcCoordinates(input_data.closest_lanelets_to_goal, last_pose);
      const double lateral_distance_to_center_lane = std::abs(path_point_arc.distance);
      const double max_target_lateral_positions = *std::max_element(
        internal_params_->sampling.target_lateral_positions.begin(),
        internal_params_->sampling.target_lateral_positions.end());
      return lateral_distance_to_center_lane / max_target_lateral_positions;
    });

  // 添加软约束：路径曲率成本
  soft_constraints_.emplace_back(
    [](
      autoware::sampler_common::Path & path,
      [[maybe_unused]] const autoware::sampler_common::Constraints & constraints,
      [[maybe_unused]] const SoftConstraintsInputs & input_data) -> double {
      if (path.curvatures.empty()) return std::numeric_limits<double>::max();

      const double max_path_curvature = *std::max_element(
        path.curvatures.begin(), path.curvatures.end(),
        [](const auto & a, const auto & b) { return std::abs(a) < std::abs(b); });
      return std::abs(max_path_curvature) / constraints.hard.max_curvature;
    });
}

// 检查是否需要执行规划
bool SamplingPlannerModule::isExecutionRequested() const
{
  // 提取车辆当前位姿
  const auto ego_pose = planner_data_->self_odometry->pose.pose;
  lanelet::ConstLanelet current_lane;

  // 获取最近的Lane
  if (!planner_data_->route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"),
      "failed to find closest lanelet within route!!!");
    return false;
  }

  // 如果之前没有输出，直接返回
  if (getPreviousModuleOutput().reference_path.points.empty()) {
    return false;
  }

  // 判断之前的输出是否是前向路径
  if (!autoware::motion_utils::isDrivingForward(getPreviousModuleOutput().reference_path.points)) {
    RCLCPP_WARN(getLogger(), "Backward path is NOT supported. Just converting path to trajectory");
    return false;
  }

  // 判断参考路径是否安全
  return !isReferencePathSafe();
}

// 检查参考路径是否安全
bool SamplingPlannerModule::isReferencePathSafe() const
{
  // TODO(Daniel): Don't use reference path, use a straight path forward.
  std::vector<DrivableLanes> drivable_lanes{};

  // 提取参考路径
  const auto & prev_module_reference_path =
    std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  // 提取参数和自车位姿
  const auto & p = planner_data_->parameters;
  const auto ego_pose = planner_data_->self_odometry->pose.pose;
  lanelet::ConstLanelet current_lane;

  // 提取最近的lane
  if (!planner_data_->route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"),
      "failed to find closest lanelet within route!!!");
    return {};
  }

  // 提取前后一定距离的lane序列
  const auto current_lane_sequence = planner_data_->route_handler->getLaneletSequence(
    current_lane, ego_pose, p.backward_path_length, p.forward_path_length);

  // 搜索附近所有可行的lane
  std::for_each(
    current_lane_sequence.begin(), current_lane_sequence.end(), [&](const auto & lanelet) {
      drivable_lanes.push_back(generateExpandDrivableLanes(lanelet, planner_data_));
    });

  lanelet::ConstLanelets current_lanes;

  // 提取所有可行的lane
  for (auto & d : drivable_lanes) {
    current_lanes.push_back(d.right_lane);
    current_lanes.push_back(d.left_lane);
    current_lanes.insert(current_lanes.end(), d.middle_lanes.begin(), d.middle_lanes.end());
  }

  {
    // 计算左右边界
    const auto path_for_calculating_bounds = getPreviousModuleOutput().reference_path;
    const auto left_bound = (utils::calcBound(
      path_for_calculating_bounds, planner_data_, drivable_lanes, false, false, false, true));
    const auto right_bound = (utils::calcBound(
      path_for_calculating_bounds, planner_data_, drivable_lanes, false, false, false, false));

    // 创建采样数据
    const auto sampling_planner_data =
      createPlannerData(planner_data_->prev_output_path, left_bound, right_bound);

    // ??
    prepareConstraints(
      internal_params_->constraints, planner_data_->dynamic_object,
      sampling_planner_data.left_bound, sampling_planner_data.right_bound);
  }

  // 将Path转为sampling_path类型
  auto transform_to_sampling_path = [](const PlanResult plan) {
    autoware::sampler_common::Path path;
    for (size_t i = 0; i < plan->points.size(); ++i) {
      const auto x = plan->points[i].point.pose.position.x;
      const auto y = plan->points[i].point.pose.position.y;
      const auto quat = plan->points[i].point.pose.orientation;
      geometry_msgs::msg::Vector3 rpy;
      tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
      tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
      path.points.emplace_back(Point2d{x, y});
      path.poses.emplace_back(plan->points[i].point.pose);
      path.yaws.emplace_back(rpy.z);
    }
    return path;
  };
  autoware::sampler_common::Path reference_path =
    transform_to_sampling_path(prev_module_reference_path);

  // 创建footprint
  const auto footprint = autoware::sampler_common::constraints::buildFootprintPoints(
    reference_path, internal_params_->constraints);

  // 判断路径是否安全
  HardConstraintsFunctionVector hard_constraints_reference_path;
  hard_constraints_reference_path.emplace_back(
    [](
      autoware::sampler_common::Path & path,
      const autoware::sampler_common::Constraints & constraints,
      const MultiPoint2d & footprint) -> bool {
      path.constraint_results.collision_free =
        !autoware::sampler_common::constraints::has_collision(
          footprint, constraints.obstacle_polygons);
      return path.constraint_results.collision_free;
    });
  evaluateHardConstraints(
    reference_path, internal_params_->constraints, footprint, hard_constraints_reference_path);
  return reference_path.constraints_satisfied;
}

// 检查是否准备好执行规划
bool SamplingPlannerModule::isExecutionReady() const
{
  return true;
}

// 创建规划器数据
SamplingPlannerData SamplingPlannerModule::createPlannerData(
  const PlanResult & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound) const
{
  SamplingPlannerData data;
  auto points = path->points;
  data.left_bound = left_bound;
  data.right_bound = right_bound;
  data.ego_pose = planner_data_->self_odometry->pose.pose;
  data.ego_vel = planner_data_->self_odometry->twist.twist.linear.x;
  return data;
}

// 将Frenet路径转换为带有车道ID的路径
PathWithLaneId SamplingPlannerModule::convertFrenetPathToPathWithLaneID(
  const autoware::frenet_planner::Path frenet_path, const lanelet::ConstLanelets & lanelets,
  const double path_z)
{
  // 从rpy转quat
  auto quaternion_from_rpy = [](double roll, double pitch, double yaw) -> tf2::Quaternion {
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    return quaternion_tf2;
  };

  PathWithLaneId path;
  const auto header = planner_data_->route_handler->getRouteHeader();
  const auto reference_path_ptr =
    std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  // 遍历所有frenet点
  for (size_t i = 0; i < frenet_path.points.size(); ++i) {

    // 提取位置和偏航
    const auto & frenet_path_point_position = frenet_path.points.at(i);
    const auto & frenet_path_point_yaw = frenet_path.yaws.at(i);

    // 赋值xyz
    PathPointWithLaneId point{};
    point.point.pose.position.x = frenet_path_point_position.x();
    point.point.pose.position.y = frenet_path_point_position.y();
    point.point.pose.position.z = path_z;

    // 赋值四元数
    auto yaw_as_quaternion = quaternion_from_rpy(0.0, 0.0, frenet_path_point_yaw);
    point.point.pose.orientation.w = yaw_as_quaternion.getW();
    point.point.pose.orientation.x = yaw_as_quaternion.getX();
    point.point.pose.orientation.y = yaw_as_quaternion.getY();
    point.point.pose.orientation.z = yaw_as_quaternion.getZ();

    // 将包含路径点的车道ID放入lane_ids中
    bool is_in_lanes = false;
    for (const auto & lane : lanelets) {
      if (lanelet::utils::isInLanelet(point.point.pose, lane)) {
        point.lane_ids.push_back(lane.id());
        is_in_lanes = true;
      }
    }

    // 如果没有对应的车道，则使用前一个路径点的车道ID
    if (!is_in_lanes && i > 0) {
      point.lane_ids = path.points.at(i - 1).lane_ids;
    }

    // 赋值速度
    if (reference_path_ptr) {
      const auto idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        reference_path_ptr->points, point.point.pose);
      const auto & closest_point = reference_path_ptr->points[idx];
      point.point.longitudinal_velocity_mps = closest_point.point.longitudinal_velocity_mps;
      point.point.lateral_velocity_mps = closest_point.point.lateral_velocity_mps;
    }

    // 插入一个PathPointWithLaneId
    path.points.push_back(point);
  }
  return path;
}

// 准备约束条件
void SamplingPlannerModule::prepareConstraints(
  autoware::sampler_common::Constraints & constraints,
  const PredictedObjects::ConstSharedPtr & predicted_objects,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound) const
{
  // 障碍物区域
  constraints.obstacle_polygons = autoware::sampler_common::MultiPolygon2d();
  constraints.rtree.clear();
  size_t i = 0;
  for (const auto & o : predicted_objects->objects) {
    if (o.kinematics.initial_twist_with_covariance.twist.linear.x < 0.5) {
      const auto polygon = autoware_utils::to_polygon2d(o);
      constraints.obstacle_polygons.push_back(polygon);
      const auto box = boost::geometry::return_envelope<autoware_utils::Box2d>(polygon);
      constraints.rtree.insert(std::make_pair(box, i));
    }
    i++;
  }

  constraints.dynamic_obstacles = {};  // TODO(Maxime): 未实现

  // TODO(Maxime): 直接使用线段而不是多边形

  // 可行驶区域
  autoware::sampler_common::Polygon2d drivable_area_polygon;
  for (const auto & p : right_bound) {
    drivable_area_polygon.outer().emplace_back(p.x, p.y);
  }
  for (auto it = left_bound.rbegin(); it != left_bound.rend(); ++it)
    drivable_area_polygon.outer().emplace_back(it->x, it->y);

  if (drivable_area_polygon.outer().size() < 1) {
    return;
  }
  drivable_area_polygon.outer().push_back(drivable_area_polygon.outer().front());

  constraints.drivable_polygons = {drivable_area_polygon};
}

// 规划路径
BehaviorModuleOutput SamplingPlannerModule::plan()
{
  // 获取参考线
  const auto reference_path_ptr =
    std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);
  if (reference_path_ptr->points.empty()) {
    return {};
  }

  // lambda: 虽然是spline，其实是提取xy
  auto reference_spline = [&]() -> autoware::sampler_common::transform::Spline2D {
    std::vector<double> x;
    std::vector<double> y;
    x.reserve(reference_path_ptr->points.size());
    y.reserve(reference_path_ptr->points.size());
    for (const auto & point : reference_path_ptr->points) {
      x.push_back(point.point.pose.position.x);
      y.push_back(point.point.pose.position.y);
    }
    return {x, y};
  }();

  autoware::frenet_planner::FrenetState frenet_initial_state;
  autoware::frenet_planner::SamplingParameters sampling_parameters;

  // 获取当前位姿
  const auto & pose = planner_data_->self_odometry->pose.pose;

  // 将当前位姿转换为另一种形式
  autoware::sampler_common::State initial_state = getInitialState(pose, reference_spline);

  // 获取采样参数
  sampling_parameters =
    prepareSamplingParameters(initial_state, reference_spline, *internal_params_);

  // lambda: 提取frenet初始状态
  auto set_frenet_state = [](
                            const autoware::sampler_common::State & initial_state,
                            const autoware::sampler_common::transform::Spline2D & reference_spline,
                            autoware::frenet_planner::FrenetState & frenet_initial_state)

  {
    frenet_initial_state.position = initial_state.frenet;
    const auto frenet_yaw = initial_state.heading - reference_spline.yaw(initial_state.frenet.s);
    const auto path_curvature = reference_spline.curvature(initial_state.frenet.s);
    constexpr auto delta_s = 0.001;
    frenet_initial_state.lateral_velocity =
      (1 - path_curvature * initial_state.frenet.d) * std::tan(frenet_yaw);
    const auto path_curvature_deriv =
      (reference_spline.curvature(initial_state.frenet.s + delta_s) - path_curvature) / delta_s;
    const auto cos_yaw = std::cos(frenet_yaw);
    if (cos_yaw == 0.0) {
      frenet_initial_state.lateral_acceleration = 0.0;
    } else {
      frenet_initial_state.lateral_acceleration =
        -(path_curvature_deriv * initial_state.frenet.d +
          path_curvature * frenet_initial_state.lateral_velocity) *
          std::tan(frenet_yaw) +
        ((1 - path_curvature * initial_state.frenet.d) / (cos_yaw * cos_yaw)) *
          (initial_state.curvature * ((1 - path_curvature * initial_state.frenet.d) / cos_yaw) -
           path_curvature);
    }
  };

  // 将当前状态映射到frenet坐标系
  set_frenet_state(initial_state, reference_spline, frenet_initial_state);

  // 提取path
  const auto prev_module_path = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().path);

  const auto & p = planner_data_->parameters;
  const auto ego_pose = planner_data_->self_odometry->pose.pose;
  lanelet::ConstLanelet current_lane;

  // 提取当前Lane
  if (!planner_data_->route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"),
      "failed to find closest lanelet within route!!!");
    return getPreviousModuleOutput();
  }

  // 提取周围的lane
  std::vector<DrivableLanes> drivable_lanes{};
  const auto current_lane_sequence = planner_data_->route_handler->getLaneletSequence(
    current_lane, ego_pose, p.backward_path_length, p.forward_path_length);
  // expand drivable lanes

  std::for_each(
    current_lane_sequence.begin(), current_lane_sequence.end(), [&](const auto & lanelet) {
      drivable_lanes.push_back(generateExpandDrivableLanes(lanelet, planner_data_));
    });

  lanelet::ConstLanelets current_lanes;

  // 进一步提取lane
  for (auto & d : drivable_lanes) {
    current_lanes.push_back(d.right_lane);
    current_lanes.push_back(d.left_lane);
    current_lanes.insert(current_lanes.end(), d.middle_lanes.begin(), d.middle_lanes.end());
  }

  // 提取目标位姿
  auto get_goal_pose = [&]() {
    auto goal_pose = planner_data_->route_handler->getGoalPose();
    if (!std::any_of(current_lanes.begin(), current_lanes.end(), [&](const auto & lane) {
          return lanelet::utils::isInLanelet(goal_pose, lane);
        })) {
      for (auto it = reference_path_ptr->points.rbegin(); it < reference_path_ptr->points.rend();
           it++) {
        if (std::any_of(current_lanes.begin(), current_lanes.end(), [&](const auto & lane) {
              return lanelet::utils::isInLanelet(it->point.pose, lane);
            })) {
          goal_pose = it->point.pose;
          break;
        }
      }
    }
    return goal_pose;
  };

  const bool prev_path_is_valid = prev_sampling_path_ && prev_sampling_path_->points.size() > 0;
  const int path_divisions = internal_params_->sampling.previous_path_reuse_points_nb;
  const bool is_extend_previous_path = path_divisions > 0;

  std::vector<autoware::frenet_planner::Path> frenet_paths;
  // Extend prev path

  // 如果之前的路径合理，并且需要延展frenet_path
  if (prev_path_is_valid && is_extend_previous_path) {

    // 提取之前的frenet数据
    autoware::frenet_planner::Path prev_path_frenet = prev_sampling_path_.value();
    frenet_paths.push_back(prev_path_frenet);

    // lambda: 提取begin()到begin() + offset的子路径
    auto get_subset = [](
                        const autoware::frenet_planner::Path & path,
                        size_t offset) -> autoware::frenet_planner::Path {
      autoware::frenet_planner::Path s;
      s.points = {path.points.begin(), path.points.begin() + offset};
      s.curvatures = {path.curvatures.begin(), path.curvatures.begin() + offset};
      s.yaws = {path.yaws.begin(), path.yaws.begin() + offset};
      s.lengths = {path.lengths.begin(), path.lengths.begin() + offset};
      s.poses = {path.poses.begin(), path.poses.begin() + offset};
      return s;
    };

    // 获取初始位姿
    autoware::sampler_common::State current_state;
    current_state.pose = {ego_pose.position.x, ego_pose.position.y};

    // 计算之前frenet路径和当前位姿的最近索引值
    const auto closest_iter = std::min_element(
      prev_path_frenet.points.begin(), prev_path_frenet.points.end() - 1,
      [&](const auto & p1, const auto & p2) {
        return boost::geometry::distance(p1, current_state.pose) <=
               boost::geometry::distance(p2, current_state.pose);
      });

      // 
    const auto current_idx = std::distance(prev_path_frenet.points.begin(), closest_iter);
    const double current_length = prev_path_frenet.lengths.at(current_idx);
    const double remaining_path_length = prev_path_frenet.lengths.back() - current_length;
    const double length_step = remaining_path_length / path_divisions;

    // 从前往后遍历获取可重复使用的路径
    for (double reuse_length = 0.0; reuse_length <= remaining_path_length;
         reuse_length += length_step) {
      size_t reuse_idx;
      for (reuse_idx = current_idx + 1; reuse_idx + 2 < prev_path_frenet.lengths.size();
           ++reuse_idx) {
        if (prev_path_frenet.lengths[reuse_idx] - current_length >= reuse_length) break;
      }

      const auto reused_path = get_subset(prev_path_frenet, reuse_idx);

      // 计算断点
      geometry_msgs::msg::Pose future_pose = reused_path.poses.back();
      autoware::sampler_common::State future_state = getInitialState(future_pose, reference_spline);
      autoware::frenet_planner::FrenetState frenet_reuse_state;

      set_frenet_state(future_state, reference_spline, frenet_reuse_state);
      autoware::frenet_planner::SamplingParameters extension_sampling_parameters =
        prepareSamplingParameters(future_state, reference_spline, *internal_params_);
      
      // 从断电开始计算剩余的frenet路径 （核心算法）
      auto extension_frenet_paths = autoware::frenet_planner::generatePaths(
        reference_spline, frenet_reuse_state, extension_sampling_parameters);

      // 将剩余的frenet路径插入到frenet_paths
      for (auto & path : extension_frenet_paths) {
        if (!path.points.empty()) frenet_paths.push_back(reused_path.extend(path));
      }
    }
  } 
  // 如果之前的路径不存在，直接生成新的frenet_path （核心算法）
  else {
    frenet_paths = autoware::frenet_planner::generatePaths(
      reference_spline, frenet_initial_state, sampling_parameters);
  }

  // 再次提取参考路径，并提取左右边界
  const auto path_for_calculating_bounds = getPreviousModuleOutput().reference_path;
  const auto left_bound = (utils::calcBound(
    path_for_calculating_bounds, planner_data_, drivable_lanes, false, false, false, true));
  const auto right_bound = (utils::calcBound(
    path_for_calculating_bounds, planner_data_, drivable_lanes, false, false, false, false));

  // 创建采样路径
  const auto sampling_planner_data =
    createPlannerData(planner_data_->prev_output_path, left_bound, right_bound);

  // 准备约束
  prepareConstraints(
    internal_params_->constraints, planner_data_->dynamic_object, sampling_planner_data.left_bound,
    sampling_planner_data.right_bound);

  // 计算软性约束输入？
  SoftConstraintsInputs soft_constraints_input;
  const auto & goal_pose = get_goal_pose();
  soft_constraints_input.goal_pose = soft_constraints_input.ego_pose =
    planner_data_->self_odometry->pose.pose;

  soft_constraints_input.current_lanes = current_lanes;
  soft_constraints_input.reference_path = reference_path_ptr;
  soft_constraints_input.prev_module_path = prev_module_path;

  soft_constraints_input.ego_arc = lanelet::utils::getArcCoordinates(current_lanes, ego_pose);
  soft_constraints_input.goal_arc = lanelet::utils::getArcCoordinates(current_lanes, goal_pose);
  lanelet::ConstLanelet closest_lanelet_to_goal;
  lanelet::utils::query::getClosestLanelet(current_lanes, goal_pose, &closest_lanelet_to_goal);
  soft_constraints_input.closest_lanelets_to_goal = {closest_lanelet_to_goal};

  debug_data_.footprints.clear();

  // 评估frenet_path
  std::vector<std::vector<double>> soft_constraints_results_full;
  for (auto & path : frenet_paths) {
    const auto footprint = autoware::sampler_common::constraints::buildFootprintPoints(
      path, internal_params_->constraints);
    evaluateHardConstraints(path, internal_params_->constraints, footprint, hard_constraints_);
    path.constraint_results.valid_curvature = true;
    debug_data_.footprints.push_back(footprint);
    std::vector<double> soft_constraints_results = evaluateSoftConstraints(
      path, internal_params_->constraints, soft_constraints_, soft_constraints_input);
    soft_constraints_results_full.push_back(soft_constraints_results);
  }

  // 将frenet_paths深复制到candidate_paths
  std::vector<autoware::sampler_common::Path> candidate_paths;
  const auto move_to_paths = [&candidate_paths](auto & paths_to_move) {
    candidate_paths.insert(
      candidate_paths.end(), std::make_move_iterator(paths_to_move.begin()),
      std::make_move_iterator(paths_to_move.end()));
  };
  move_to_paths(frenet_paths);

  // 使用candidate_paths更新debug数据
  debug_data_.previous_sampled_candidates_nb = debug_data_.sampled_candidates.size();
  debug_data_.sampled_candidates = candidate_paths;
  debug_data_.obstacles = internal_params_->constraints.obstacle_polygons;
  updateDebugMarkers();

  // 筛选出代价最低的frenet_path
  const double max_length = *std::max_element(
    internal_params_->sampling.target_lengths.begin(),
    internal_params_->sampling.target_lengths.end());
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data_, max_length, max_length, false);  // Do these max lengths make sense?

  const auto best_path_idx = [](const auto & paths) {
    auto min_cost = std::numeric_limits<double>::max();
    size_t best_path_idx = 0;
    for (auto i = 0LU; i < paths.size(); ++i) {
      if (paths[i].constraints_satisfied && paths[i].cost < min_cost) {
        best_path_idx = i;
        min_cost = paths[i].cost;
      }
    }
    return min_cost < std::numeric_limits<double>::max() ? std::optional<size_t>(best_path_idx)
                                                         : std::nullopt;
  };
  const auto selected_path_idx = best_path_idx(frenet_paths);

  // 如果没有代价最低的frenet_path
  if (!selected_path_idx) {
    // 直接输出上一条结果
    BehaviorModuleOutput out;
    PathWithLaneId out_path = (prev_sampling_path_)
                                ? convertFrenetPathToPathWithLaneID(
                                    prev_sampling_path_.value(), current_lanes,
                                    planner_data_->route_handler->getGoalPose().position.z)
                                : PathWithLaneId{};

    out.path = (prev_sampling_path_) ? out_path : getPreviousModuleOutput().path;
    out.reference_path = getPreviousModuleOutput().reference_path;
    out.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
    extendOutputDrivableArea(out, drivable_lanes);
    return out;
  }

  // 提取代价最低的frenet_path
  const auto best_path = frenet_paths[*selected_path_idx];

  // 打印信息
  std::cerr << "Soft constraints results of best: ";
  for (const auto result : soft_constraints_results_full[*selected_path_idx])
    std::cerr << result << ",";
  std::cerr << "\n";

  std::cerr << "Poses " << best_path.poses.size() << "\n";
  std::cerr << "Length of best " << best_path.lengths.back() << "\n";

  const auto out_path = convertFrenetPathToPathWithLaneID(
    best_path, current_lanes, planner_data_->route_handler->getGoalPose().position.z);
  prev_sampling_path_ = best_path;

  std::cerr << "road_lanes size " << road_lanes.size() << "\n";
  std::cerr << "First lane ID size " << out_path.points.at(0).lane_ids.size() << "\n";

  // 填充结果
  BehaviorModuleOutput out;
  out.path = out_path;
  out.reference_path = *reference_path_ptr;
  out.drivable_area_info = getPreviousModuleOutput().drivable_area_info;

  // 扩展可行使区域
  extendOutputDrivableArea(out, drivable_lanes);

  // 返回结果
  return out;
}

// 更新调试标记
void SamplingPlannerModule::updateDebugMarkers()
{
  debug_marker_.markers.clear();
  info_marker_.markers.clear();

  const auto header = planner_data_->route_handler->getRouteHeader();
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = header.stamp;
  m.action = m.ADD;
  m.id = 0UL;
  m.type = m.LINE_STRIP;
  m.color.a = 1.0;
  m.scale.x = 0.02;
  m.ns = "candidates";
  for (const auto & c : debug_data_.sampled_candidates) {
    m.points.clear();
    for (const auto & p : c.points)
      m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
    if (c.constraint_results.isValid()) {
      m.color.g = 1.0;
      m.color.r = 0.0;
    } else {
      m.color.r = 1.0;
      m.color.g = 0.0;
    }
    debug_marker_.markers.push_back(m);
    info_marker_.markers.push_back(m);
    ++m.id;
  }
  m.ns = "footprint";
  m.id = 0UL;
  m.type = m.POINTS;
  m.points.clear();
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 1.0;
  m.scale.y = 0.2;
  if (!debug_data_.footprints.empty()) {
    m.action = m.ADD;
    for (const auto & p : debug_data_.footprints[0]) {
      m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
    }

  } else {
    m.action = m.DELETE;
  }
  debug_marker_.markers.push_back(m);
  info_marker_.markers.push_back(m);
  ++m.id;
  m.type = m.LINE_STRIP;
  m.ns = "obstacles";
  m.id = 0UL;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  for (const auto & obs : debug_data_.obstacles) {
    m.points.clear();
    for (const auto & p : obs.outer())
      m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
    debug_marker_.markers.push_back(m);
    info_marker_.markers.push_back(m);
    ++m.id;
  }
  m.action = m.DELETE;
  m.ns = "candidates";
  for (m.id = debug_data_.sampled_candidates.size();
       static_cast<size_t>(m.id) < debug_data_.previous_sampled_candidates_nb; ++m.id) {
    debug_marker_.markers.push_back(m);
    info_marker_.markers.push_back(m);
  }
}

// 扩展输出可行驶区域
void SamplingPlannerModule::extendOutputDrivableArea(
  BehaviorModuleOutput & output, std::vector<DrivableLanes> & drivable_lanes)
{
  // 用于新架构
  DrivableAreaInfo current_drivable_area_info;
  current_drivable_area_info.drivable_lanes = drivable_lanes;
  output.drivable_area_info = utils::combineDrivableAreaInfo(
    current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);
}

// 规划候选路径
CandidateOutput SamplingPlannerModule::planCandidate() const
{
  return {};
}

// 更新数据
void SamplingPlannerModule::updateData()
{
}

// 工具函数

template <typename T>
void pushUniqueVector(T & base_vector, const T & additional_vector)
{
  base_vector.insert(base_vector.end(), additional_vector.begin(), additional_vector.end());
}

// 检查车道终点是否连接
bool SamplingPlannerModule::isEndPointsConnected(
  const lanelet::ConstLanelet & left_lane, const lanelet::ConstLanelet & right_lane) const
{
  const auto & left_back_point_2d = right_lane.leftBound2d().back().basicPoint();
  const auto & right_back_point_2d = left_lane.rightBound2d().back().basicPoint();

  constexpr double epsilon = 1e-5;
  return (right_back_point_2d - left_back_point_2d).norm() < epsilon;
}

// 生成扩展的可行驶车道
DrivableLanes SamplingPlannerModule::generateExpandDrivableLanes(
  const lanelet::ConstLanelet & lanelet,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto & route_handler = planner_data->route_handler;

  DrivableLanes current_drivable_lanes;
  current_drivable_lanes.left_lane = lanelet;
  current_drivable_lanes.right_lane = lanelet;

  // 1. 获取左/右侧车道
  const auto update_left_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto all_left_lanelets =
      route_handler->getAllLeftSharedLinestringLanelets(target_lane, true, true);
    if (!all_left_lanelets.empty()) {
      current_drivable_lanes.left_lane = all_left_lanelets.back();  // leftmost lanelet
      pushUniqueVector(
        current_drivable_lanes.middle_lanes,
        lanelet::ConstLanelets(all_left_lanelets.begin(), all_left_lanelets.end() - 1));
    }
  };
  const auto update_right_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto all_right_lanelets =
      route_handler->getAllRightSharedLinestringLanelets(target_lane, true, true);
    if (!all_right_lanelets.empty()) {
      current_drivable_lanes.right_lane = all_right_lanelets.back();  // rightmost lanelet
      pushUniqueVector(
        current_drivable_lanes.middle_lanes,
        lanelet::ConstLanelets(all_right_lanelets.begin(), all_right_lanelets.end() - 1));
    }
  };

  update_left_lanelets(lanelet);
  update_right_lanelets(lanelet);

  // 2.1 当存在多个车道的前一个车道相同时
  const auto get_next_lanes_from_same_previous_lane =
    [&route_handler](const lanelet::ConstLanelet & lane) {
      // get previous lane, and return false if previous lane does not exist
      lanelet::ConstLanelets prev_lanes;
      if (!route_handler->getPreviousLaneletsWithinRoute(lane, &prev_lanes)) {
        return lanelet::ConstLanelets{};
      }

      lanelet::ConstLanelets next_lanes;
      for (const auto & prev_lane : prev_lanes) {
        const auto next_lanes_from_prev = route_handler->getNextLanelets(prev_lane);
        pushUniqueVector(next_lanes, next_lanes_from_prev);
      }
      return next_lanes;
    };

  const auto next_lanes_for_right =
    get_next_lanes_from_same_previous_lane(current_drivable_lanes.right_lane);
  const auto next_lanes_for_left =
    get_next_lanes_from_same_previous_lane(current_drivable_lanes.left_lane);

  // 2.2 递归查找相邻车道，其中车道的终点与原始车道的终点相连
  const auto update_drivable_lanes =
    [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
      for (const auto & next_lane : next_lanes) {
        const auto & edge_lane =
          is_left ? current_drivable_lanes.left_lane : current_drivable_lanes.right_lane;
        if (next_lane.id() == edge_lane.id()) {
          continue;
        }

        const auto & left_lane = is_left ? next_lane : edge_lane;
        const auto & right_lane = is_left ? edge_lane : next_lane;
        if (!isEndPointsConnected(left_lane, right_lane)) {
          continue;
        }

        if (is_left) {
          current_drivable_lanes.left_lane = next_lane;
        } else {
          current_drivable_lanes.right_lane = next_lane;
        }

        const auto & middle_lanes = current_drivable_lanes.middle_lanes;
        const auto has_same_lane = std::any_of(
          middle_lanes.begin(), middle_lanes.end(),
          [&edge_lane](const auto & lane) { return lane.id() == edge_lane.id(); });

        if (!has_same_lane) {
          if (is_left) {
            if (current_drivable_lanes.right_lane.id() != edge_lane.id()) {
              current_drivable_lanes.middle_lanes.push_back(edge_lane);
            }
          } else {
            if (current_drivable_lanes.left_lane.id() != edge_lane.id()) {
              current_drivable_lanes.middle_lanes.push_back(edge_lane);
            }
          }
        }

        return true;
      }
      return false;
    };

  const auto expand_drivable_area_recursively =
    [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
      // 注意：设置最大搜索次数以避免无限循环
      constexpr size_t max_recursive_search_num = 3;
      for (size_t i = 0; i < max_recursive_search_num; ++i) {
        const bool is_update_kept = update_drivable_lanes(next_lanes, is_left);
        if (!is_update_kept) {
          break;
        }
        if (i == max_recursive_search_num - 1) {
          RCLCPP_ERROR(
            rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
            "Drivable area expansion reaches max iteration.");
        }
      }
    };
  expand_drivable_area_recursively(next_lanes_for_right, false);
  expand_drivable_area_recursively(next_lanes_for_left, true);

  // 3. 再次更新新的左/右车道
  update_left_lanelets(current_drivable_lanes.left_lane);
  update_right_lanelets(current_drivable_lanes.right_lane);

  // 4. 确保当前车道在左车道、右车道或中间车道中
  if (
    current_drivable_lanes.left_lane.id() != lanelet.id() &&
    current_drivable_lanes.right_lane.id() != lanelet.id()) {
    current_drivable_lanes.middle_lanes.push_back(lanelet);
  }

  return current_drivable_lanes;
}

// 准备采样参数
autoware::frenet_planner::SamplingParameters SamplingPlannerModule::prepareSamplingParameters(
  const autoware::sampler_common::State & initial_state,
  const autoware::sampler_common::transform::Spline2D & path_spline,
  const SamplingPlannerInternalParameters & params_)
{
  // 计算目标横向位置
  std::vector<double> target_lateral_positions;
  if (params_.sampling.nb_target_lateral_positions > 1) {
    target_lateral_positions = {0.0, initial_state.frenet.d};
    double min_d = 0.0;
    double max_d = 0.0;
    double min_d_s = std::numeric_limits<double>::max();
    double max_d_s = std::numeric_limits<double>::max();
    for (const auto & drivable_poly : params_.constraints.drivable_polygons) {
      for (const auto & p : drivable_poly.outer()) {
        const auto frenet_coordinates = path_spline.frenet(p);
        const auto d_s = std::abs(frenet_coordinates.s - initial_state.frenet.s);
        if (d_s < min_d_s && frenet_coordinates.d < 0.0) {
          min_d_s = d_s;
          min_d = frenet_coordinates.d;
        }
        if (d_s < max_d_s && frenet_coordinates.d > 0.0) {
          max_d_s = d_s;
          max_d = frenet_coordinates.d;
        }
      }
    }
    min_d += params_.constraints.ego_width / 2.0;
    max_d -= params_.constraints.ego_width / 2.0;
    if (min_d < max_d) {
      for (auto r = 0.0; r <= 1.0; r += 1.0 / (params_.sampling.nb_target_lateral_positions - 1))
        target_lateral_positions.push_back(autoware::interpolation::lerp(min_d, max_d, r));
    }
  } else {
    target_lateral_positions = params_.sampling.target_lateral_positions;
  }
  autoware::frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.resolution = params_.sampling.resolution;
  const auto max_s = path_spline.lastS();
  autoware::frenet_planner::SamplingParameter p;
  for (const auto target_length : params_.sampling.target_lengths) {
    p.target_state.position.s =
      std::min(max_s, path_spline.frenet(initial_state.pose).s + std::max(0.0, target_length));
    for (const auto target_lateral_position : target_lateral_positions) {
      p.target_state.position.d = target_lateral_position;
      for (const auto target_lat_vel : params_.sampling.frenet.target_lateral_velocities) {
        p.target_state.lateral_velocity = target_lat_vel;
        for (const auto target_lat_acc : params_.sampling.frenet.target_lateral_accelerations) {
          p.target_state.lateral_acceleration = target_lat_acc;
          sampling_parameters.parameters.push_back(p);
        }
      }
    }
    if (p.target_state.position.s == max_s) break;
  }
  return sampling_parameters;
}

}  // namespace autoware::behavior_path_planner
