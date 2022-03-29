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

#include "opendrive/parser/TrafficSignalsParser.h"

#include <iostream>

void opendrive::parser::TrafficSignalsParser::Parse(
    const pugi::xml_node &xmlNode,
    std::vector<opendrive::TrafficSignalInformation> &out_traffic_signals,
    std::vector<opendrive::TrafficSignalReference>
        &out_traffic_signal_references) {
  for (pugi::xml_node signal = xmlNode.child("signal"); signal;
       signal = signal.next_sibling("signal")) {
    opendrive::TrafficSignalInformation trafficSignalInformation;

    trafficSignalInformation.id = std::atoi(signal.attribute("id").value());

    trafficSignalInformation.start_position =
        std::stod(signal.attribute("s").value());
    trafficSignalInformation.track_position =
        std::stod(signal.attribute("t").value());

    trafficSignalInformation.zoffset =
        std::stod(signal.attribute("zOffset").value());

    trafficSignalInformation.value = signal.attribute("value").as_double();

    trafficSignalInformation.name = signal.attribute("name").value();
    trafficSignalInformation.dynamic = signal.attribute("dynamic").value();
    trafficSignalInformation.orientation =
        signal.attribute("orientation").value();

    trafficSignalInformation.type = signal.attribute("type").value();
    trafficSignalInformation.subtype = signal.attribute("subtype").value();
    trafficSignalInformation.country = signal.attribute("country").value();

    out_traffic_signals.emplace_back(trafficSignalInformation);
  }

  for (pugi::xml_node signal = xmlNode.child("signalReference"); signal;
       signal = signal.next_sibling("signalReference")) {
    opendrive::TrafficSignalReference trafficSignalReference;

    trafficSignalReference.id = std::atoi(signal.attribute("id").value());
    trafficSignalReference.start_position =
        std::stod(signal.attribute("s").value());
    trafficSignalReference.track_position =
        std::stod(signal.attribute("t").value());
    trafficSignalReference.orientation =
        signal.attribute("orientation").value();
    out_traffic_signal_references.emplace_back(trafficSignalReference);
  }
}
