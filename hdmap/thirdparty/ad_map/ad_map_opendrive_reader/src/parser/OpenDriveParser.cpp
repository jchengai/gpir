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

#include "opendrive/parser/OpenDriveParser.hpp"

#include <iostream>
#include <pugixml.hpp>

#include "opendrive/parser/GeoReferenceParser.h"
#include "opendrive/parser/GeometryParser.h"
#include "opendrive/parser/JunctionParser.h"
#include "opendrive/parser/LaneParser.h"
#include "opendrive/parser/ProfilesParser.h"
#include "opendrive/parser/RoadLinkParser.h"
#include "opendrive/parser/TrafficGroupParser.h"
#include "opendrive/parser/TrafficSignParser.h"
#include "opendrive/parser/TrafficSignalsParser.h"

namespace opendrive {
namespace parser {

bool OpenDriveParser::Parse(const char *xml,
                            opendrive::OpenDriveData &out_open_drive_data,
                            XmlInputType inputType, std::string *out_error) {
  namespace odp = opendrive::parser;

  pugi::xml_document xmlDoc;
  pugi::xml_parse_result pugiParseResult;

  unsigned int const parse_options_ensure_no_doctype =
      pugi::parse_default & ~pugi::parse_doctype;
  switch (inputType) {
    case XmlInputType::FILE: {
      pugiParseResult = xmlDoc.load_file(xml, parse_options_ensure_no_doctype);
    } break;

    case XmlInputType::CONTENT: {
      pugiParseResult =
          xmlDoc.load_string(xml, parse_options_ensure_no_doctype);
    } break;

    default: {
      // TODO: Log some kind of error
      return false;
    } break;
  }

  if (pugiParseResult == false) {
    if (out_error != nullptr) {
      *out_error = pugiParseResult.description();
    }

    return false;
  }

  for (pugi::xml_node road = xmlDoc.child("OpenDRIVE").child("road"); road;
       road = road.next_sibling("road")) {
    opendrive::RoadInformation openDriveRoadInformation;

    openDriveRoadInformation.attributes.name = road.attribute("name").value();
    openDriveRoadInformation.attributes.id =
        std::atoi(road.attribute("id").value());
    openDriveRoadInformation.attributes.length =
        std::stod(road.attribute("length").value());
    openDriveRoadInformation.attributes.junction =
        std::atoi(road.attribute("junction").value());

    // types
    for (pugi::xml_node node_type : road.children("type")) {
      RoadTypeInfo type{0.0, ""};

      type.s = node_type.attribute("s").as_double();
      type.type = node_type.attribute("type").value();
      openDriveRoadInformation.attributes.type.emplace_back(type);

      // speed type
      pugi::xml_node speed_node = node_type.child("speed");
      if (speed_node) {
        RoadSpeed speed{0.0, 0.0, ""};
        speed.s = type.s;
        speed.max = speed_node.attribute("max").as_double();
        speed.unit = speed_node.attribute("unit").value();
        openDriveRoadInformation.attributes.speed.emplace_back(speed);
      }
    }

    ///////////////////////////////////////////////////////////////////////////////

    odp::ProfilesParser::Parse(road, openDriveRoadInformation.road_profiles);

    odp::RoadLinkParser::Parse(road.child("link"),
                               openDriveRoadInformation.road_link);
    odp::TrafficSignalsParser::Parse(
        road.child("signals"), openDriveRoadInformation.traffic_signals,
        openDriveRoadInformation.traffic_signal_references);

    odp::LaneParser::Parse(road.child("lanes"), openDriveRoadInformation.lanes);
    odp::GeometryParser::Parse(road.child("planView"),
                               openDriveRoadInformation.geometry_attributes);

    out_open_drive_data.roads.emplace_back(std::move(openDriveRoadInformation));
  }

  for (pugi::xml_node junction = xmlDoc.child("OpenDRIVE").child("junction");
       junction; junction = junction.next_sibling("junction")) {
    odp::JunctionParser::Parse(junction, out_open_drive_data.junctions);
  }

  for (pugi::xml_node tlgroup = xmlDoc.child("OpenDRIVE").child("tlGroup");
       tlgroup; tlgroup = tlgroup.next_sibling("tlGroup")) {
    odp::TrafficGroupParser::Parse(tlgroup,
                                   out_open_drive_data.trafficlightgroups);
  }

  for (pugi::xml_node trafficsigns =
           xmlDoc.child("OpenDRIVE").child("trafficsign");
       trafficsigns; trafficsigns = trafficsigns.next_sibling("trafficsign")) {
    odp::TrafficSignParser::Parse(trafficsigns,
                                  out_open_drive_data.trafficsigns);
  }

  out_open_drive_data.geoReference = odp::GeoReferenceParser::Parse(
      xmlDoc.child("OpenDRIVE").child("header").child_value("geoReference"));

  return true;
}
}  // namespace parser
}  // namespace opendrive
