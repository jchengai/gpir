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

#include <pugixml.hpp>

#include "opendrive/types.hpp"

namespace opendrive {
namespace parser {

class ProfilesParser {
 private:
  void ParseElevation(
      const pugi::xml_node &xmlNode,
      std::vector<opendrive::ElevationProfile> &out_elevation_profile);

  void ParseLateral(
      const pugi::xml_node &xmlNode,
      std::vector<opendrive::LateralProfile> &out_lateral_profile);

 public:
  static void Parse(const pugi::xml_node &xmlNode,
                    opendrive::RoadProfiles &out_road_profiles);
};
}  // namespace parser
}  // namespace opendrive
