add_library(planning_core_carla)
add_library(planning_core::carla ALIAS planning_core_carla)

add_dependencies(planning_core_carla carla_msgs_gencpp)

target_sources(planning_core_carla PRIVATE
  carla_adapter.cc
  ego_info/carla_ego_info.cc
  mock_perception/carla_mock_perception.cc
)

target_include_directories(planning_core_carla PUBLIC
  ${roscpp_INCLUDE_DIRS}
  ${tf_INCLUDE_DIRS}
  ${msgs_INCLUDE_DIRS}
)

target_link_libraries(planning_core_carla
  glog
  ${roscpp_LIBRARIES}
  ${tf_LIBRARIES}
  planning_core::common
)

add_executable(carla_tf carla_tf.cc)
target_include_directories(carla_tf PRIVATE
  ${roscpp_INCLUDE_DIRS}
  ${tf_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)
target_link_libraries(carla_tf
  ${tf_LIBRARIES}
  ${tf2_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_executable(carla_controller carla_controller.cc)
add_dependencies(carla_controller carla_msgs_gencpp)
target_include_directories(carla_controller PRIVATE
  ${ackermann_msgs_INCLUDE_DIRS}
  ${carla_msgs_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)
target_link_libraries(carla_controller 
  ${roscpp_LIBRARIES}
)