add_library(hdmap_utils)
add_library(hdmap::utils ALIAS hdmap_utils)

target_sources(hdmap_utils PRIVATE
  admap_visual.cc
  admap_convertion.cc
)
target_include_directories(hdmap_utils PUBLIC
  ${ADMAP_INCLUDE_DIRS}
)
target_link_libraries(hdmap_utils
  common::utils
  ad_map_access
  ad_physics
)