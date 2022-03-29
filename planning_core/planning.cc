/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/planning_core.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "planning");
  ros::NodeHandle node;

  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::INFO);
  // FLAGS_log_prefix = false
  FLAGS_colorlogtostderr = true;

  planning::PlanningCore planning_core;
  planning_core.Init();

  ros::Timer main_loop = node.createTimer(
      ros::Duration(0.05), &planning::PlanningCore::Run, &planning_core);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
