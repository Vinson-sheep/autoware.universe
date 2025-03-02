// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef GOAL_POSE_VISUALIZER__GOAL_POSE_VISUALIZER_HPP_
#define GOAL_POSE_VISUALIZER__GOAL_POSE_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace autoware::mission_planner_universe
{

// 这个主要作用是负责可视化目标位置。它订阅路由消息并发布目标位置的可视化信息。
// 主要和与 mission_planner.cpp 和 route_selector.cpp 交互，接收路由信息并发布目标位置。
class GoalPoseVisualizer : public rclcpp::Node
{
public:
  explicit GoalPoseVisualizer(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;

  void echo_back_route_callback(
    const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg);
};

}  // namespace autoware::mission_planner_universe
#endif  // GOAL_POSE_VISUALIZER__GOAL_POSE_VISUALIZER_HPP_
