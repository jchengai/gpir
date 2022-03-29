# find_package(osqp REQUIRED)

add_library(planning_core_controller)
add_library(planning_core::controller ALIAS planning_core_controller)

target_sources(planning_core_controller PRIVATE
  mpc_controller.cc
  lmpc_osqp_solver.cc
)
target_include_directories(planning_core_controller PUBLIC
  ${ackermann_msgs_INCLUDE_DIRS}
)
target_link_libraries(planning_core_controller
  # osqp::osqp
  common::solver
)