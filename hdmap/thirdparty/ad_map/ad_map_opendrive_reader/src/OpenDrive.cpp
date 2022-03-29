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

#include "opendrive/OpenDrive.hpp"

#include "opendrive/geometry/GeometryGenerator.hpp"
#include "opendrive/parser/OpenDriveParser.hpp"

namespace opendrive {

bool Load(std::string const &file, opendrive::OpenDriveData &open_drive_data) {
  return parser::OpenDriveParser::Parse(file.c_str(), open_drive_data,
                                        parser::XmlInputType::FILE);
}

bool Parse(std::string const &content,
           opendrive::OpenDriveData &open_drive_data) {
  return parser::OpenDriveParser::Parse(content.c_str(), open_drive_data,
                                        parser::XmlInputType::CONTENT);
}

bool GenerateLaneMap(opendrive::OpenDriveData &open_drive_data,
                     double const overlapMargin) {
  return geometry::GenerateGeometry(open_drive_data, overlapMargin);
}
}  // namespace opendrive
