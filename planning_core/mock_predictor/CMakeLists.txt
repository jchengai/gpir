add_library(planning_core_mock_predictor)
add_library(planning_core::mock_predictor ALIAS planning_core_mock_predictor)

target_sources(planning_core_mock_predictor PRIVATE
  mock_predictor.cc
  const_vel_predictor.cc
)
target_link_libraries(planning_core_mock_predictor
  planning_core::common
  hdmap::hdmap
)