set(osqp_DIR ${PROJECT_SOURCE_DIR}/thirdparty/osqp/install/lib/cmake/osqp)

find_package(osqp REQUIRED)

add_library(common_solver)
add_library(common::solver ALIAS common_solver)

target_sources(common_solver PRIVATE
  osqp/osqp_interface.cc
)
target_include_directories(common_solver PUBLIC
  ${PROJECT_SOURCE_DIR}/thirdparty/osqp/install/include
)

target_link_libraries(common_solver 
  glog
  osqp::osqpstatic
)
