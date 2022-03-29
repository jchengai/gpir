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

#include "opendrive/types.hpp"

#include <pugixml.hpp>

namespace opendrive {
namespace parser {

class JunctionParser
{
private:
  void ParseConnection(const pugi::xml_node &xmlNode, std::vector<opendrive::JunctionConnection> &out_connections);

  void ParseLaneLink(const pugi::xml_node &xmlNode, std::vector<opendrive::JunctionLaneLink> &out_lane_link);

public:
  static void Parse(const pugi::xml_node &xmlNode, std::vector<opendrive::Junction> &out_junction);
};
}
}
