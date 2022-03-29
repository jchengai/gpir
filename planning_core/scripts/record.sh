# use "rosparam set /use_sim_time true" before replay the bag

rosbag record /tf \
              /tf_static \
              /clock \
              /rosout \
              /hdmap \
              /hdmap_topo \
              /hdmap_full_route \
              /carla/ego_vehicle/odometry \
              /carla/ego_vehicle/vehicle_status \
              /carla/ego_vehicle/vehicle_info \
              /carla/objects \
              /joint_states \
              /move_base_simple/goal \
              /obstacle_bbox \
              /obstacle_info \
              /prediction \
              /critical_obstacles \
              /virtual_obstacle \
              /reference_line \
              /gp_path \
              /behavior_target_lane \
              /joy
