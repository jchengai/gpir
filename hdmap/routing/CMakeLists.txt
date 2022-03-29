add_library(hdmap_routing)
add_library(hdmap::routing ALIAS hdmap_routing)

target_sources(hdmap_routing PRIVATE
  routing.cc
  full_route.cc
) 
target_link_libraries(hdmap_routing
  glog
  ad_map_access
)