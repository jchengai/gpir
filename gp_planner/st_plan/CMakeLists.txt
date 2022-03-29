add_library(gp_planner_st_plan)
add_library(gp_planner::st_plan ALIAS gp_planner_st_plan)

target_sources(gp_planner_st_plan PRIVATE
  st_graph.cc
  st_node.cc
)

target_link_libraries(gp_planner_st_plan
  gp_planner::gp
  gp_planner::sdf
)

