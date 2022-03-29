add_library(common_smoothing)
add_library(common::smoothing ALIAS common_smoothing)

target_sources(common_smoothing
  PRIVATE
    affine_constraint.cc
    discrete_points_math.cc
    osqp_spline1d_solver.cc
    osqp_spline2d_solver.cc
    polynomialxd.cc
    spline1d_constraint.cc
    spline1d_kernel_helper.cc
    spline1d_kernel.cc
    spline1d_seg.cc
    spline1d.cc
    spline2d_constraint.cc
    spline2d_kernel.cc
    spline2d_seg
    spline2d.cc
)

target_link_libraries(common_smoothing 
  glog
  common_solver
)