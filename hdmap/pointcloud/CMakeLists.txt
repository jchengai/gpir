find_package(catkin REQUIRED COMPONENTS
  pcl_ros
)

add_library(hdmap_pointcloud)
add_library(hdmap::pointcloud ALIAS hdmap_pointcloud)

target_sources(hdmap_pointcloud PRIVATE
  pointcloud_loader.cc
)
target_include_directories(hdmap_pointcloud PRIVATE
  ${pcl_ros_INCLUDE_DIRS}
)
target_link_libraries(hdmap_pointcloud PRIVATE
  ${pcl_ros_LIBRARIES}
)