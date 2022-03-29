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

#include "opendrive/parser/GeometryParser.h"

#include <cassert>

#define ODP_ASSERT(x, ...) assert(x)
#define ODP_UNUSED(x) (void)(x)

void opendrive::parser::GeometryParser::ParseArc(
    const pugi::xml_node &xmlNode,
    opendrive::GeometryAttributesArc *out_geometry_arc) {
  out_geometry_arc->type = opendrive::GeometryType::ARC;
  out_geometry_arc->curvature =
      std::stod(xmlNode.attribute("curvature").value());
}

void opendrive::parser::GeometryParser::ParseLine(
    const pugi::xml_node &xmlNode,
    opendrive::GeometryAttributesLine *out_geometry_line) {
  ODP_UNUSED(xmlNode);
  out_geometry_line->type = opendrive::GeometryType::LINE;
}

void opendrive::parser::GeometryParser::ParseSpiral(
    const pugi::xml_node &xmlNode,
    opendrive::GeometryAttributesSpiral *out_geometry_spiral) {
  out_geometry_spiral->type = opendrive::GeometryType::SPIRAL;
  out_geometry_spiral->curve_end =
      std::stod(xmlNode.attribute("curvEnd").value());
  out_geometry_spiral->curve_start =
      std::stod(xmlNode.attribute("curvStart").value());
}

void opendrive::parser::GeometryParser::ParsePoly3(
    const pugi::xml_node &xmlNode,
    opendrive::GeometryAttributesPoly3 *out_geometry_poly3) {
  out_geometry_poly3->type = opendrive::GeometryType::POLY3;
  out_geometry_poly3->a = std::stod(xmlNode.attribute("a").value());
  out_geometry_poly3->b = std::stod(xmlNode.attribute("b").value());
  out_geometry_poly3->c = std::stod(xmlNode.attribute("c").value());
  out_geometry_poly3->d = std::stod(xmlNode.attribute("d").value());
}

void opendrive::parser::GeometryParser::ParseParamPoly3(
    const pugi::xml_node &xmlNode,
    opendrive::GeometryAttributesParamPoly3 *out_geometry_param_poly3) {
  out_geometry_param_poly3->type = opendrive::GeometryType::PARAMPOLY3;
  out_geometry_param_poly3->aU = std::stod(xmlNode.attribute("aU").value());
  out_geometry_param_poly3->bU = std::stod(xmlNode.attribute("bU").value());
  out_geometry_param_poly3->cU = std::stod(xmlNode.attribute("cU").value());
  out_geometry_param_poly3->dU = std::stod(xmlNode.attribute("dU").value());
  out_geometry_param_poly3->aV = std::stod(xmlNode.attribute("aV").value());
  out_geometry_param_poly3->bV = std::stod(xmlNode.attribute("bV").value());
  out_geometry_param_poly3->cV = std::stod(xmlNode.attribute("cV").value());
  out_geometry_param_poly3->dV = std::stod(xmlNode.attribute("dV").value());
}

void opendrive::parser::GeometryParser::Parse(
    const pugi::xml_node &xmlNode,
    std::vector<std::unique_ptr<opendrive::GeometryAttributes>>
        &out_geometry_attributes) {
  opendrive::parser::GeometryParser gometry_parser;

  for (pugi::xml_node roadGeometry = xmlNode.child("geometry"); roadGeometry;
       roadGeometry = roadGeometry.next_sibling("geometry")) {
    std::unique_ptr<opendrive::GeometryAttributes> geometry_attributes;
    std::string firstChildName(roadGeometry.first_child().name());

    if (firstChildName == "arc") {
      geometry_attributes =
          std::make_unique<opendrive::GeometryAttributesArc>();
      gometry_parser.ParseArc(roadGeometry.first_child(),
                              static_cast<opendrive::GeometryAttributesArc *>(
                                  geometry_attributes.get()));
    } else if (firstChildName == "line") {
      geometry_attributes =
          std::make_unique<opendrive::GeometryAttributesLine>();
      gometry_parser.ParseLine(roadGeometry.first_child(),
                               static_cast<opendrive::GeometryAttributesLine *>(
                                   geometry_attributes.get()));
    } else if (firstChildName == "spiral") {
      geometry_attributes =
          std::make_unique<opendrive::GeometryAttributesSpiral>();
      gometry_parser.ParseSpiral(
          roadGeometry.first_child(),
          static_cast<opendrive::GeometryAttributesSpiral *>(
              geometry_attributes.get()));
    } else if (firstChildName == "poly3") {
      geometry_attributes =
          std::make_unique<opendrive::GeometryAttributesPoly3>();
      gometry_parser.ParsePoly3(
          roadGeometry.first_child(),
          static_cast<opendrive::GeometryAttributesPoly3 *>(
              geometry_attributes.get()));
    } else if (firstChildName == "paramPoly3") {
      geometry_attributes =
          std::make_unique<opendrive::GeometryAttributesParamPoly3>();
      gometry_parser.ParseParamPoly3(
          roadGeometry.first_child(),
          static_cast<opendrive::GeometryAttributesParamPoly3 *>(
              geometry_attributes.get()));
    } else {
      ODP_ASSERT(false, "Geometry type unknown");
    }

    geometry_attributes->start_position =
        std::stod(roadGeometry.attribute("s").value());

    geometry_attributes->start_position_x =
        std::stod(roadGeometry.attribute("x").value());
    geometry_attributes->start_position_y =
        std::stod(roadGeometry.attribute("y").value());

    geometry_attributes->heading =
        std::stod(roadGeometry.attribute("hdg").value());
    geometry_attributes->length =
        std::stod(roadGeometry.attribute("length").value());

    out_geometry_attributes.emplace_back(std::move(geometry_attributes));
  }
}
