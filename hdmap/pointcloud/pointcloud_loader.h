/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <string>

#include "sensor_msgs/PointCloud2.h"

namespace hdmap {
class PointCloudLoader {
 public:
  static bool LoadPcdFile(const std::string& pcd_file,
                          sensor_msgs::PointCloud2* pointcloud);
};
}  // namespace hdmap
