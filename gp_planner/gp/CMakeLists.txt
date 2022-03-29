add_library(gp_planner_gp)
add_library(gp_planner::gp ALIAS gp_planner_gp)

target_sources(gp_planner_gp
  PRIVATE
    factors/gp_prior_factor.cc
    factors/gp_obstacle_factor.cc
    factors/gp_kappa_limit_factor.cc
    factors/gp_lat_acc_limit_factor.cc
    factors/gp_interpolate_obstacle_factor.cc
    factors/gp_interpolate_kappa_limit_factor.cc
    utils/gp_utils.cc
    utils/gp_path.cc
    gp_incremental_path_planner.cc
)

target_link_libraries(gp_planner_gp PUBLIC
  gtsam
  planning_core::navigation
  common::smoothing
  gp_planner::initializer
)