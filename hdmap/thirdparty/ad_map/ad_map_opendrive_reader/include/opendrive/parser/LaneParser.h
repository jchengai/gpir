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

class LaneParser
{
private:
  void ParseLane(const pugi::xml_node &xmlNode, std::vector<LaneInfo> &out_lane);

  void ParseLaneSpeed(const pugi::xml_node &xmlNode, std::vector<LaneSpeed> &out_lane_speed);

  void ParseLaneLink(const pugi::xml_node &xmlNode, std::unique_ptr<LaneLink> &out_lane_link);

  void ParseLaneOffset(const pugi::xml_node &xmlNode, std::vector<LaneOffset> &out_lane_offset);

  void ParseLaneWidth(const pugi::xml_node &xmlNode, std::vector<LaneWidth> &out_lane_width);

  void ParseLaneRoadMark(const pugi::xml_node &xmlNode, std::vector<LaneRoadMark> &out_lane_mark);

public:
  static void Parse(const pugi::xml_node &xmlNode, Lanes &out_lanes);
};

} // parser
} // opendrive
