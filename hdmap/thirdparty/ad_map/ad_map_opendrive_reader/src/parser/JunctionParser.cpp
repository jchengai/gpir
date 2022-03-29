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

#include "opendrive/parser/JunctionParser.h"

void opendrive::parser::JunctionParser::Parse(
    const pugi::xml_node &xmlNode,
    std::vector<opendrive::Junction> &out_junction) {
  opendrive::parser::JunctionParser parser;
  opendrive::Junction junction;

  junction.attributes.id = std::atoi(xmlNode.attribute("id").value());
  junction.attributes.name = xmlNode.attribute("name").value();

  parser.ParseConnection(xmlNode, junction.connections);
  out_junction.emplace_back(junction);
}

void opendrive::parser::JunctionParser::ParseConnection(
    const pugi::xml_node &xmlNode,
    std::vector<opendrive::JunctionConnection> &out_connections) {
  for (pugi::xml_node junctionConnection = xmlNode.child("connection");
       junctionConnection;
       junctionConnection = junctionConnection.next_sibling("connection")) {
    opendrive::JunctionConnection jConnection;

    jConnection.attributes.id =
        std::atoi(junctionConnection.attribute("id").value());
    jConnection.attributes.contact_point =
        junctionConnection.attribute("contactPoint").value();

    jConnection.attributes.incoming_road =
        std::atoi(junctionConnection.attribute("incomingRoad").value());
    jConnection.attributes.connecting_road =
        std::atoi(junctionConnection.attribute("connectingRoad").value());

    ParseLaneLink(junctionConnection, jConnection.links);
    out_connections.emplace_back(jConnection);
  }
}

void opendrive::parser::JunctionParser::ParseLaneLink(
    const pugi::xml_node &xmlNode,
    std::vector<opendrive::JunctionLaneLink> &out_lane_link) {
  for (pugi::xml_node junctionLaneLink = xmlNode.child("laneLink");
       junctionLaneLink;
       junctionLaneLink = junctionLaneLink.next_sibling("laneLink")) {
    opendrive::JunctionLaneLink jLaneLink;

    jLaneLink.from = std::atoi(junctionLaneLink.attribute("from").value());
    jLaneLink.to = std::atoi(junctionLaneLink.attribute("to").value());

    out_lane_link.emplace_back(jLaneLink);
  }
}
