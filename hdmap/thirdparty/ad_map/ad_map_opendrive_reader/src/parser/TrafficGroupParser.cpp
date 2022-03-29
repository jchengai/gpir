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

#include "opendrive/parser/TrafficGroupParser.h"

#include <iostream>

void opendrive::parser::TrafficGroupParser::Parse(
    const pugi::xml_node &xmlNode,
    std::vector<opendrive::TrafficLightGroup> &out_trafficLights) {
  opendrive::parser::TrafficGroupParser parser;
  opendrive::TrafficLightGroup traffic_light_group;

  traffic_light_group.red_time =
      std::atoi(xmlNode.attribute("redTime").value());
  traffic_light_group.yellow_time =
      std::atoi(xmlNode.attribute("yellowTime").value());
  traffic_light_group.green_time =
      std::atoi(xmlNode.attribute("greenTime").value());

  parser.ParseTrafficLight(xmlNode, traffic_light_group.traffic_lights);
  out_trafficLights.emplace_back(traffic_light_group);
}

void opendrive::parser::TrafficGroupParser::ParseTrafficLight(
    const pugi::xml_node &xmlNode,
    std::vector<opendrive::TrafficLight> &out_trafficLight) {
  for (pugi::xml_node trafficlight = xmlNode.child("trafficlight");
       trafficlight; trafficlight = trafficlight.next_sibling("trafficlight")) {
    opendrive::TrafficLight jTrafficlight;

    jTrafficlight.x_pos = std::stod(trafficlight.attribute("xPos").value());
    jTrafficlight.y_pos = std::stod(trafficlight.attribute("yPos").value());
    jTrafficlight.z_pos = std::stod(trafficlight.attribute("zPos").value());
    jTrafficlight.x_rot = std::stod(trafficlight.attribute("xRot").value());
    jTrafficlight.y_rot = std::stod(trafficlight.attribute("yRot").value());
    jTrafficlight.z_rot = std::stod(trafficlight.attribute("zRot").value());

    ParseBoxAreas(trafficlight, jTrafficlight.box_areas);

    out_trafficLight.emplace_back(jTrafficlight);
  }
}

void opendrive::parser::TrafficGroupParser::ParseBoxAreas(
    const pugi::xml_node &xmlNode,
    std::vector<opendrive::BoxComponent> &out_boxcomponent) {
  for (pugi::xml_node boxcomponent = xmlNode.child("tfBox"); boxcomponent;
       boxcomponent = boxcomponent.next_sibling("tfBox")) {
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
