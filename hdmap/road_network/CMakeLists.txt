add_library(hdmap_road_network)
add_library(hdmap::road_network ALIAS hdmap_road_network)

target_sources(hdmap_road_network PRIVATE
  lane.cc
  lane_map.cc
  road_network.cc
)
target_include_directories(hdmap_road_network PUBLIC
  ${ADMAP_INCLUDE_DIRS}
)