add_library(common_geometry)
add_library(common::geometry ALIAS common_geometry)

target_sources(common_geometry PRIVATE
  box2d.cc
)