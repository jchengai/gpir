add_library(common_utils)
add_library(common::utils ALIAS common_utils)

target_sources(common_utils PRIVATE
  math.cc
  color_map.cc
  common_visual.cc
  io_utils.cc
)
target_include_directories(common_utils PUBLIC
  ${std_msgs_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)
target_link_libraries(common_utils
  ${roscpp_LIBRARIES}
)