/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
 * de Barcelona (UAB).
 * Copyright (C) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

#pragma once

#include <opendrive/types.hpp>
#include <string>

namespace opendrive {

bool Load(std::string const &file, opendrive::OpenDriveData &open_drive_data);
bool Parse(std::string const &content,
           opendrive::OpenDriveData &open_drive_data);
/**
 * @brief Generates the full lane map
 *
 * @param[in] overlapMargin margin the lanes are narrowed when calculating
 * overlaps.
 *
 */
bool GenerateLaneMap(opendrive::OpenDriveData &open_drive_data,
                     double const overlapMargin);
}  // namespace opendrive
