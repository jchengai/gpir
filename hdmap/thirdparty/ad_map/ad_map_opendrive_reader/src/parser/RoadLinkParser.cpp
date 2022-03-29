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

#include "opendrive/parser/RoadLinkParser.h"

#include <cstdlib>

opendrive::ElementType toElementType(std::string const elementType) {
  if (elementType == "junction") {
    return opendrive::ElementType::Junction;
  } else if (elementType == "road") {
    return opendrive::ElementType::Road;
  }

  return opendrive::ElementType::Invalid;
}

opendrive::ContactPoint toContactPoint(std::string const contactPoint) {
  if (contactPoint == "start") {
    return opendrive::ContactPoint::Start;
  } else if (contactPoint == "end") {
    return opendrive::ContactPoint::End;
  }

  return opendrive::ContactPoint::Invalid;
}

void opendrive::parser::RoadLinkParser::ParseLink(
    const pugi::xml_node &xmlNode,
    opendrive::RoadLinkInformation *out_link_information) {
  out_link_information->id = std::atoi(xmlNode.attribute("elementId").value());
  out_link_information->element_type =
      toElementType(xmlNode.attribute("elementType").value());
  out_link_information->contact_point =
      toContactPoint(xmlNode.attribute("contactPoint").value());
}

void opendrive::parser::RoadLinkParser::Parse(
    const pugi::xml_node &xmlNode, opendrive::RoadLink &out_road_link) {
  RoadLinkParser parser;

  const pugi::xml_node predecessorNode = xmlNode.child("predecessor");
  const pugi::xml_node successorNode = xmlNode.child("successor");

  if (predecessorNode) {
    out_road_link.predecessor =
        std::make_unique<opendrive::RoadLinkInformation>();
    parser.ParseLink(predecessorNode, out_road_link.predecessor.get());
  }

  if (successorNode) {
    out_road_link.successor =
        std::make_unique<opendrive::RoadLinkInformation>();
    parser.ParseLink(successorNode, out_road_link.successor.get());
  }
}
