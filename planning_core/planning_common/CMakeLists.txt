add_library(planning_core_common)
add_library(planning_core::common ALIAS planning_core_common)

target_sources(planning_core_common PRIVATE
  obstacle.cc
  planning_visual.cc
)

target_include_directories(planning_core_common PUBLIC
  ${visualization_msgs_INCLUDE_DIRS}
)

target_link_libraries(planning_core_common
  common::utils
  common::geometry
  hdmap::road_network
)