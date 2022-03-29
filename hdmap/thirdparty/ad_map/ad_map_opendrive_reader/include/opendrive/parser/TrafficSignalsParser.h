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

class TrafficSignalsParser
{
public:
  static void Parse(const pugi::xml_node &xmlNode,
                    std::vector<opendrive::TrafficSignalInformation> &out_traffic_signals,
                    std::vector<opendrive::TrafficSignalReference> &out_traffic_signal_references);
};
}
}
