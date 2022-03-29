add_library(gp_planner_initializer)
add_library(gp_planner::initializer ALIAS gp_planner_initializer)

target_sources(gp_planner_initializer PRIVATE
  gp_initializer.cc
)
target_link_libraries(gp_planner_initializer
  common::smoothing
)