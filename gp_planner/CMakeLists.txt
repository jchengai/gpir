cmake_minimum_required(VERSION 3.13.0)
project(gp_planner CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native") # have to enable because of gtsam

set(GTSAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gtsam-4.1rc/install/lib/cmake/GTSAM)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package()

# avoid tons of warnnings inside libeigen
include_directories(SYSTEM 
  ${EIGEN3_INCLUDE_DIR}
)
include_directories(
  ${CMAKE_SOURCE_DIR}/gpir
  ${CMAKE_BINARY_DIR}/gpir
  "./thirdparty/"
)

add_subdirectory(gp)
add_subdirectory(sdf)
add_subdirectory(initializer)
add_subdirectory(st_plan)

add_library(gp_planner gp_planner.cc)
target_link_libraries(gp_planner
  gp_planner::gp
  gp_planner::sdf
  gp_planner::initializer
  gp_planner::st_plan
)