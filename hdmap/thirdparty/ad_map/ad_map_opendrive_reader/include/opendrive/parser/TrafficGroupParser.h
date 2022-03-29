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

class TrafficGroupParser
{
private:
  void ParseTrafficLight(const pugi::xml_node &xmlNode, std::vector<opendrive::TrafficLight> &out_trafficLights);

  void ParseBoxAreas(const pugi::xml_node &xmlNode, std::vector<opendrive::BoxComponent> &out_boxareas);

public:
  static void Parse(const pugi::xml_node &xmlNode, std::vector<opendrive::TrafficLightGroup> &out_trafficlightgroup);
};
}
}
