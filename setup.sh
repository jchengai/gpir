#!/bin/bash

ROOT_DIR=$(cd $(dirname "$0"); pwd)

# extract 3D model
cd ${ROOT_DIR}/planning_core/3d_model && pwd && tar -xvf model3.tar.xz

# build spdlog (dependency of ad_map)
cd ${ROOT_DIR}/hdmap/thirdparty/ad_map/spdlog
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install"
make -j4 && make install

# build gtsam (dependency of gp_planner)
cd ${ROOT_DIR}/gp_planner/thirdparty/gtsam-4.1rc
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install"
make -j6 && make install

# build osqp (dependency of gp_planner)
cd ${ROOT_DIR}/common/thirdparty/osqp
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install"
make -j4 && make install

sudo apt install -y libgoogle-glog-dev \
                    libpugixml-dev \
                    libomp-dev 

sudo apt install -y ros-${ROS_DISTRO}-ackermann-msgs \
                    ros-${ROS_DISTRO}-derived-object-msgs 

sudo apt install -y ros-${ROS_DISTRO}-jsk-recognition-msgs \
                    ros-${ROS_DISTRO}-jsk-rviz-plugins 
