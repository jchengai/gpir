/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
 * de Barcelona (UAB).
 * Copyright (C) 2019-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

#include "opendrive/parser/LaneParser.h"

#include <string>

namespace opendrive {
namespace parser {

LaneType toLaneType(std::string const laneType) {
  if (laneType == "none") {
    return LaneType::None;
  } else if (laneType == "driving") {
    return LaneType::Driving;
  } else if (laneType == "stop") {
    return LaneType::Stop;
  } else if (laneType == "shoulder") {
    return LaneType::Shoulder;
  } else if (laneType == "biking") {
    return LaneType::Biking;
  } else if (laneType == "sidewalk") {
    return LaneType::Sidewalk;
  } else if (laneType == "border") {
    return LaneType::Border;
  } else if (laneType == "restricted") {
    return LaneType::Restricted;
  } else if (laneType == "bidirectional") {
    return LaneType::Bidirectional;
  } else if (laneType == "parking") {
    return LaneType::Parking;
  } else if (laneType == "median") {
    return LaneType::Median;
  } else if (laneType == "special1") {
    return LaneType::Special1;
  } else if (laneType == "special2") {
    return LaneType::Special2;
  } else if (laneType == "special3") {
    return LaneType::Special3;
  } else if (laneType == "roadWorks") {
    return LaneType::RoadWorks;
  } else if (laneType == "tram") {
    return LaneType::Tram;
  } else if (laneType == "rail") {
    return LaneType::Rail;
  } else if (laneType == "entry") {
    return LaneType::Entry;
  } else if (laneType == "exit") {
    return LaneType::Exit;
  } else if (laneType == "offRamp") {
    return LaneType::OffRamp;
  } else if (laneType == "onRamp") {
    return LaneType::OnRamp;
  } else {
    return LaneType::None;
  }
}

void LaneParser::ParseLane(const pugi::xml_node &xmlNode,
                           std::vector<LaneInfo> &out_lane) {
  for (pugi::xml_node lane = xmlNode.child("lane"); lane;
       lane = lane.next_sibling("lane")) {
    LaneInfo currentLane;

    currentLane.attributes.type = toLaneType(lane.attribute("type").value());
    currentLane.attributes.level = lane.attribute("level").value();
    currentLane.attributes.id = std::atoi(lane.attribute("id").value());

    ParseLaneSpeed(lane, currentLane.lane_speed);
    ParseLaneWidth(lane, currentLane.lane_width);

    ParseLaneLink(lane.child("link"), currentLane.link);
    ParseLaneRoadMark(lane, currentLane.road_marker);

    out_lane.emplace_back(std::move(currentLane));
  }
}

void LaneParser::ParseLaneWidth(const pugi::xml_node &xmlNode,
                                std::vector<LaneWidth> &out_lane_width) {
  for (pugi::xml_node laneWidth = xmlNode.child("width"); laneWidth;
       laneWidth = laneWidth.next_sibling("width")) {
    LaneWidth laneWidthInfo;

    laneWidthInfo.soffset = std::stod(laneWidth.attribute("sOffset").value());

    laneWidthInfo.a = std::stod(laneWidth.attribute("a").value());
    laneWidthInfo.b = std::stod(laneWidth.attribute("b").value());

    laneWidthInfo.c = std::stod(laneWidth.attribute("c").value());
    laneWidthInfo.d = std::stod(laneWidth.attribute("d").value());

    out_lane_width.emplace_back(laneWidthInfo);
  }
}

void LaneParser::ParseLaneLink(const pugi::xml_node &xmlNode,
                               std::unique_ptr<LaneLink> &out_lane_link) {
  const pugi::xml_node predecessorNode = xmlNode.child("predecessor");
  const pugi::xml_node successorNode = xmlNode.child("successor");

  out_lane_link = (predecessorNode || successorNode)
                      ? std::make_unique<opendrive::LaneLink>()
                      : nullptr;
  if (out_lane_link == nullptr) {
    return;
  }

  out_lane_link->predecessor_id =
      predecessorNode ? std::atoi(predecessorNode.attribute("id").value()) : 0;
  out_lane_link->successor_id =
      successorNode ? std::atoi(successorNode.attribute("id").value()) : 0;
}

void LaneParser::ParseLaneOffset(const pugi::xml_node &xmlNode,
                                 std::vector<LaneOffset> &out_lane_offset) {
  LaneOffset lanesOffset;

  lanesOffset.s = std::stod(xmlNode.attribute("s").value());
  lanesOffset.a = std::stod(xmlNode.attribute("a").value());
  lanesOffset.b = std::stod(xmlNode.attribute("b").value());
  lanesOffset.c = std::stod(xmlNode.attribute("c").value());
  lanesOffset.d = std::stod(xmlNode.attribute("d").value());

  out_lane_offset.emplace_back(lanesOffset);
}

void LaneParser::ParseLaneRoadMark(const pugi::xml_node &xmlNode,
                                   std::vector<LaneRoadMark> &out_lane_mark) {
  for (pugi::xml_node road_mark = xmlNode.child("roadMark"); road_mark;
       road_mark = road_mark.next_sibling("roadMark")) {
    LaneRoadMark roadMarker;

    if (road_mark.attribute("sOffset") != nullptr) {
      roadMarker.soffset = std::stod(road_mark.attribute("sOffset").value());
    }

    if (road_mark.attribute("width") != nullptr) {
      roadMarker.width = std::stod(road_mark.attribute("width").value());
    }

    if (road_mark.attribute("type") != nullptr) {
      roadMarker.type = road_mark.attribute("type").value();
    }

    if (road_mark.attribute("weight") != nullptr) {
      roadMarker.weigth = road_mark.attribute("weight").value();
    }

    if (road_mark.attribute("material") != nullptr) {
      roadMarker.color = road_mark.attribute("material").value();
    }

    if (road_mark.attribute("color") != nullptr) {
      roadMarker.color = road_mark.attribute("color").value();
    }

    if (road_mark.attribute("laneChange") != nullptr) {
      roadMarker.lane_change = road_mark.attribute("laneChange").value();
    }

    out_lane_mark.emplace_back(roadMarker);
  }
}

void LaneParser::ParseLaneSpeed(const pugi::xml_node &xmlNode,
                                std::vector<LaneSpeed> &out_lane_speed) {
  for (pugi::xml_node laneSpeed = xmlNode.child("speed"); laneSpeed;
       laneSpeed = laneSpeed.next_sibling("speed")) {
    LaneSpeed lane_speed = {0.0, 0.0, ""};

    lane_speed.soffset = std::stod(laneSpeed.attribute("sOffset").value());
    lane_speed.max_speed = std::stod(laneSpeed.attribute("max").value());
    lane_speed.unit = laneSpeed.attribute("unit").value();

    out_lane_speed.emplace_back(lane_speed);
  }
}

void LaneParser::Parse(const pugi::xml_node &xmlNode, Lanes &out_lanes) {
  LaneParser laneParser;

  for (pugi::xml_node laneSection = xmlNode.child("laneOffset"); laneSection;
       laneSection = laneSection.next_sibling("laneOffset")) {
    laneParser.ParseLaneOffset(laneSection, out_lanes.lane_offset);
  }

  for (pugi::xml_node laneSection = xmlNode.child("laneSection"); laneSection;
       laneSection = laneSection.next_sibling("laneSection")) {
    LaneSection laneSec;
    laneSec.start_position = std::stod(laneSection.attribute("s").value());
    // until we know more, we set end to start
    laneSec.end_position = laneSec.start_position;

    pugi::xml_node lane = laneSection.child("left");
    laneParser.ParseLane(lane, laneSec.left);

    lane = laneSection.child("center");
    laneParser.ParseLane(lane, laneSec.center);

    lane = laneSection.child("right");
    laneParser.ParseLane(lane, laneSec.right);

    out_lanes.lane_sections.emplace_back(std::move(laneSec));
  }
}

}  // namespace parser
}  // namespace opendrive
