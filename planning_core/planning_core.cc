/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/planning_core.h"

#include <thread>

#include "common/utils/timer.h"
#include "gp_planner/gp_planner.h"
#include "hdmap/hdmap.h"
#include "planning_common/planning_visual.h"
#include "planning_core/simulation/simulator_adapter_factory.h"

namespace planning {

void PlanningCore::Init() {
  ros::NodeHandle node("~");

  bool load_param = true;
  std::string simulator, town, map_path;
  load_param &= node.getParam("simulator", simulator);
  load_param &= node.getParam("town", town);
  load_param &= node.getParam("map_path", map_path);
  load_param &= node.getParam("random_drive_mode", random_drive_mode_);
  if (!load_param) LOG(FATAL) << "fail to init param";

  // init hdmap
  std::string map = map_path + town + ".txt";
  std::string pcd = map_path + "pcd/" + town + ".pcd";
  if (!hdmap::HdMap::GetMap().LoadMap(map, pcd)) {
    LOG(FATAL) << "fail to init hdmap, \nmap: " << map << "\npcd: " << pcd;
  }

  // init simulator adapter
  simulator_ = simulation::SimulatorFactory::CreateSimulatorAdapter(simulator);
  simulator_->Init();
  if (!simulator_->InitVehicleParam(
          VehicleInfo::Instance().mutable_vehicle_param())) {
    LOG(FATAL) << "fail to init vehicle param from " << simulator_->Name();
  }
  LOG(INFO) << "Init simulator " << simulator_->Name() << " OK";

  // init planning
  navigation_map_.Init();
  data_frame_ = std::make_shared<DataFrame>();
  route_target_sub_ = node.subscribe("/move_base_simple/goal", 10,
                                     &PlanningCore::NewRouteCallBack, this);
  joy_sub_ = node.subscribe("/joy", 10, &PlanningCore::JoyCallBack, this);

  // init predictor
  mock_predictor_ = std::make_unique<ConstVelPredictor>(6, 0.2);
  mock_predictor_->Init();

  // init planner
  planner_ = std::make_unique<GPPlanner>();
  planner_->Init();
}

void PlanningCore::Run(const ros::TimerEvent&) {
  if (!UpdateDataFrame()) {
    LOG_EVERY_N(ERROR, 20) << "update data frame failed, give up planning";
    return;
  }

  mock_predictor_->GeneratePrediction(&data_frame_->obstacles);
  navigation_map_.Update(data_frame_);

  if (!random_drive_mode_) {
    // point-to-point mode
    std::lock_guard<std::mutex> lock(route_mutex_);
    if (has_new_route_) {
      navigation_map_.CreateTask(route_goal_);
      has_new_route_ = false;
    }
    if (!navigation_map_.HasActiveTask()) {
      LOG_EVERY_N(INFO, 20) << "no active task";
      navigation_map_.mutable_trajectory()->clear();
      simulator_->SetTrajectory(navigation_map_.trajectory());
      return;
    }
  } else {
    // random driving mode
    if (has_new_route_) {
      if (!navigation_map_.RandomlyUpdateRoute()) {
        navigation_map_.mutable_trajectory()->clear();
        simulator_->SetTrajectory(navigation_map_.trajectory());
        return;
      }
    } else {
      navigation_map_.mutable_trajectory()->clear();
      simulator_->SetTrajectory(navigation_map_.trajectory());
      return;
    }
  }

  if (suggest_lane_change_) {
    if (navigation_map_.SuggestLaneChange(
            static_cast<hdmap::LaneSegmentBehavior>(suggest_lane_change_))) {
      suggest_lane_change_ = 0;
    }
  }

  navigation_map_.UpdateReferenceLine();
  TIC;
  planner_->PlanOnce(&navigation_map_);
  TOC("PlaneOnce");
  simulator_->SetTrajectory(navigation_map_.trajectory());
}

void PlanningCore::NewRouteCallBack(const geometry_msgs::PoseStamped& goal) {
  std::lock_guard<std::mutex> lock(route_mutex_);
  route_goal_ = goal;
  has_new_route_ = true;
}

void PlanningCore::JoyCallBack(const sensor_msgs::Joy& joy) {
  if (joy.buttons[2] == 1) {
    LOG(INFO) << "Suggest left lane change by joy";
    suggest_lane_change_ = 1;
  } else if (joy.buttons[1] == 1) {
    LOG(INFO) << "Suggest right lane change by joy";
    suggest_lane_change_ = 2;
  } else if (joy.buttons[3] == 1) {
    LOG(INFO) << "Increase reference speed";
    navigation_map_.AdjustReferenceSpeed(1);
  } else if (joy.buttons[0] == 1) {
    LOG(INFO) << "Increase reference speed";
    navigation_map_.AdjustReferenceSpeed(-1);
  } else if (joy.buttons[5] > 0) {
    LOG(INFO) << "Add virtual Obstacles";
    navigation_map_.RandomlyAddVirtualObstacles(joy.buttons[5]);
  }
}

bool PlanningCore::UpdateDataFrame() {
  if (!simulator_->UpdateEgoState(&data_frame_->state)) return false;
  if (!simulator_->UpdatePerceptionResults(&data_frame_->obstacles))
    return false;
  return true;
}
}  // namespace planning
