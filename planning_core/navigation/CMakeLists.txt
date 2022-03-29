add_library(planning_core_navigation)
add_library(planning_core::navigation ALIAS planning_core_navigation)

target_sources(planning_core_navigation PRIVATE
  navigation_map.cc
  reference_line.cc
  route_sequence.cc
)
target_link_libraries(planning_core_navigation
  glog
  hdmap
  common::frenet
  common::smoothing
  planning_core::common
)