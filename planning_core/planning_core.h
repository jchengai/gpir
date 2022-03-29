/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <memory>
#include <mutex>

#include "planning_core/mock_predictor/const_vel_predictor.h"
#include "planning_core/navigation/navigation_map.h"
#include "planning_core/planner/planner.h"
#include "planning_core/planning_common/data_frame.h"
#include "planning_core/simulation/simulator_adapter.h"

namespace planning {

class PlanningCore {
 public:
  PlanningCore() = default;

  void Init();

  void Run(const ros::TimerEvent&);

 private:
  void NewRouteCallBack(const geometry_msgs::PoseStamped& goal);
  void JoyCallBack(const sensor_msgs::Joy& joy);
  bool UpdateDataFrame();

 private:
  std::mutex route_mutex_;
  bool random_drive_mode_ = false;
  bool has_new_route_ = false;
  ros::Subscriber route_target_sub_;
  geometry_msgs::PoseStamped route_goal_;

  ros::Subscriber joy_sub_;
  int suggest_lane_change_ = 0;

  NavigationMap navigation_map_;
  std::shared_ptr<DataFrame> data_frame_;
  std::unique_ptr<simulation::SimulatorAdapter> simulator_;

  std::unique_ptr<Planner> planner_;
  std::unique_ptr<MockPredictor> mock_predictor_;
};

}  // namespace planning
