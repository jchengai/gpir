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

#include <string>
#include "opendrive/types.hpp"

namespace opendrive {
namespace parser {

enum class XmlInputType : int
{
  FILE,
  CONTENT
};

struct OpenDriveParser
{
  static bool Parse(const char *xml,
                    opendrive::OpenDriveData &out_open_drive_data,
                    XmlInputType inputType,
                    std::string *out_error = nullptr);
};
}
}
