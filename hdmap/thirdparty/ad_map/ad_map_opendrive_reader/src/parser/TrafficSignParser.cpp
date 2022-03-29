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

#include "opendrive/parser/TrafficSignParser.h"

#include <iostream>

void opendrive::parser::TrafficSignParser::Parse(
    const pugi::xml_node &xmlNode,
    std::vector<opendrive::TrafficSign> &out_trafficsigns) {
  opendrive::parser::TrafficSignParser parser;
  opendrive::TrafficSign trafficsign;

  trafficsign.speed = std::atoi(xmlNode.attribute("speed").value());
  trafficsign.x_pos = std::stod(xmlNode.attribute("xPos").value());
  trafficsign.y_pos = std::stod(xmlNode.attribute("yPos").value());
  trafficsign.z_pos = std::stod(xmlNode.attribute("zPos").value());
  trafficsign.x_rot = std::stod(xmlNode.attribute("xRot").value());
  trafficsign.y_rot = std::stod(xmlNode.attribute("yRot").value());
  trafficsign.z_rot = std::stod(xmlNode.attribute("zRot").value());

  parser.ParseBoxAreas(xmlNode, trafficsign.box_areas);
  out_trafficsigns.emplace_back(trafficsign);
}

void opendrive::parser::TrafficSignParser::ParseBoxAreas(
    const pugi::xml_node &xmlNode,
    std::vector<opendrive::BoxComponent> &out_boxcomponent) {
  for (pugi::xml_node boxcomponent = xmlNode.child("tsBox"); boxcomponent;
       boxcomponent = boxcomponent.next_sibling("tsBox")) {
    opendrive::BoxComponent jBoxComponent;

    jBoxComponent.x_pos = std::stod(boxcomponent.attribute("xPos").value());
    jBoxComponent.y_pos = std::stod(boxcomponent.attribute("yPos").value());
    jBoxComponent.z_pos = std::stod(boxcomponent.attribute("zPos").value());
    jBoxComponent.x_rot = std::stod(boxcomponent.attribute("xRot").value());
    jBoxComponent.y_rot = std::stod(boxcomponent.attribute("yRot").value());
    jBoxComponent.z_rot = std::stod(boxcomponent.attribute("zRot").value());

    out_boxcomponent.emplace_back(jBoxComponent);
  }
}
