/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <fstream>
#include <utility>

namespace common {

void DotLog(std::ofstream& os);

template <typename T, typename... Args>
void DotLog(std::ofstream& os, T&& data, Args&&... args) {
  os << std::forward<T>(data) << ",";
  DotLog(os, std::forward<Args>(args)...);
}

}  // namespace common
