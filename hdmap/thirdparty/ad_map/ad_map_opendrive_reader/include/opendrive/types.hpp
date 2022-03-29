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

#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace opendrive {

enum class GeometryType : unsigned int { ARC, LINE, SPIRAL, POLY3, PARAMPOLY3 };

enum class LaneType : std::uint8_t {
  None,
  Driving,
  Stop,
  Shoulder,
  Biking,
  Sidewalk,
  Border,
  Restricted,
  Parking,
  Bidirectional,
  Median,
  Special1,
  Special2,
  Special3,
  RoadWorks,
  Tram,
  Rail,
  Entry,
  Exit,
  OffRamp,
  OnRamp
};

enum class RoadType : std::uint8_t {
  Unknown,
  Tural,
  Motorway,
  Town,
  LowSpeed,
  Pedestrian,
  Bicycle
};

enum class RoadmarkType : std::uint8_t {
  None,
  Solid,
  Broken,
  SolidSolid,
  SolidBroken,
  BrokenSolid,
  BrokenBroken,
  BottsDots,
  Grass,
  Curb
};

enum class ElementType : std::uint8_t { Invalid, Junction, Road };

enum class ContactPoint : std::uint8_t { Invalid, Start, End };

struct Point {
  double x{0.};
  double y{0.};

  Point(double _x, double _y) : x(_x), y(_y) {}

  bool operator==(const Point &rhs) const {
    // Points are treated as equal when below 1 mm
    return (std::fabs(x - rhs.x) < 1e-3) && (std::fabs(y - rhs.y) < 1e-3);
  }
  bool operator!=(const Point &rhs) const { return !operator==(rhs); }

  double normSquared() const { return x * x + y * y; }

  double norm() const { return sqrt(normSquared()); }

  double dot(const Point &other) const { return x * other.x + y * other.y; }
};

inline Point operator-(const Point &left, const Point &right) {
  return Point(left.x - right.x, left.y - right.y);
}

struct GeometryAttributes {
  GeometryType type;  // geometry type
  double length;      // length of the road section
                      // [meters]

  double start_position;  // s-offset [meters]
  double heading;         // start orientation [radians]

  double start_position_x;  // [meters]
  double start_position_y;  // [meters]
};

struct GeometryAttributesArc : public GeometryAttributes {
  double curvature{0.};
};

struct GeometryAttributesLine : public GeometryAttributes {
  // Nothing else here
};

struct GeometryAttributesSpiral : public GeometryAttributes {
  double curve_start{0.};
  double curve_end{0.};
};

struct GeometryAttributesPoly3 : public GeometryAttributes {
  double a{0.};
  double b{0.};
  double c{0.};
  double d{0.};
};

struct GeometryAttributesParamPoly3 : public GeometryAttributes {
  double aU{0.};
  double bU{0.};
  double cU{0.};
  double dU{0.};
  double aV{0.};
  double bV{0.};
  double cV{0.};
  double dV{0.};
  std::string p_range{"arcLength"};
};

/////////////////////////////////////////////////////////////////

struct LaneAttributes {
  int id;
  LaneType type;
  std::string level;
};

struct LaneWidth {
  double soffset;  // start position (s-offset) [meters]
  double a;        // a - width [meters]
  double b;        // b
  double c;        // c
  double d;        // d
};

struct LaneRoadMark {
  double soffset = 0.0;
  double width = 0.0;

  std::string type;
  std::string weigth = "standard";

  // See OpenDRIVE Format Specification, Rev. 1.4
  // Doc No.: VI2014.107 (5.3.7.2.1.1.4 Road Mark Record)
  std::string material = "standard";

  std::string color = "white";
  std::string lane_change = "none";
};

struct LaneOffset {
  double s, a, b, c, d;
};

struct LaneSpeed {
  double soffset;    // start position(s - offset from the
                     // current lane section) [meters]
  double max_speed;  // maximum allowed speed [meters/second]
  std::string unit;
};

struct LaneLink {
  int predecessor_id;
  int successor_id;
};

struct ParametricSpeed {
  double start{0.};
  double end{1.};
  double speed{0.};

  ParametricSpeed() : start{0.}, end{1.0}, speed{0.} {}
  ParametricSpeed(double _speed) : start{0.}, end{1.0}, speed{_speed} {}
};

struct TrafficSignalReference {
  int id{-1};
  double start_position{0.};  // s
  double track_position{0.};  // t
  std::string orientation{""};
};

struct SignalReference {
  int id{-1};
  double parametricPosition{0.0};
  bool inLaneOrientation{false};
};

using Id = uint64_t;
using Edge = std::vector<Point>;

struct LaneInfo {
  std::vector<LaneSpeed> lane_speed;

  LaneAttributes attributes;
  std::vector<LaneWidth> lane_width;

  std::vector<LaneRoadMark> road_marker;
  std::unique_ptr<LaneLink> link;
};

struct LaneSection {
  double start_position{0.};
  double end_position{0.};  // extended value
  std::vector<LaneInfo> left, center, right;
};

struct Lanes {
  std::vector<LaneOffset> lane_offset;
  std::vector<LaneSection> lane_sections;
};

/////////////////////////////////////////////////////////////////

struct ElevationProfile {
  double start_position;      // (S) start position(s -
                              // offset)[meters]
  double elevation;           // (A) elevation [meters]
  double slope;               // (B)
  double vertical_curvature;  // (C)
  double curvature_change;    // (D)
};

struct LateralProfile {
  double start_position;      // (S) start position(s -
                              // offset)[meters]
  double elevation;           // (A) elevation [meters]
  double slope;               // (B)
  double vertical_curvature;  // (C)
  double curvature_change;    // (D)
};

struct RoadProfiles {
  std::vector<ElevationProfile> elevation_profile;
  std::vector<LateralProfile> lateral_profile;
};

/////////////////////////////////////////////////////////////////

struct TrafficSignalInformation {
  int id;

  double start_position;  // s
  double track_position;  // t

  double zoffset;  // z offset from track level
  double value;    // value of the signal (e.g. speed,
                   // mass � depending on type)

  std::string name;         // name of the signal (e.g. gfx bead
                            // name)
  std::string dynamic;      // boolean identification whether
                            // signal is a dynamic
                            // signal(e.g.traffic light)
  std::string orientation;  // "+" = valid in positive track
                            // direction; "-" = valid in
                            // negative track direction; "none"
                            // = valid in both directions

  std::string country;  // country code of the signa
  std::string type;     // type identifier according to
                        // country code or "-1" / "none"
  std::string subtype;  // subtype identifier according to
                        // country code or "-1" / "none"
};

/////////////////////////////////////////////////////////////////

struct RoadTypeInfo {
  double s;
  std::string type;
};

struct RoadSpeed {
  double s;
  double max;
  std::string unit;
};

struct RoadAttributes {
  std::string name;
  int id, junction;
  double length;
  std::vector<RoadTypeInfo> type;
  std::vector<RoadSpeed> speed;

  RoadAttributes() : id(-1), junction(-1), length(0.0) {}
};

struct RoadLinkInformation {
  int id;
  ElementType element_type;
  ContactPoint contact_point;

  RoadLinkInformation()
      : id(-1),
        element_type(ElementType::Invalid),
        contact_point(ContactPoint::Invalid) {}
};

struct RoadLink {
  std::unique_ptr<RoadLinkInformation> successor;
  std::unique_ptr<RoadLinkInformation> predecessor;
};

struct RoadInformation {
  RoadLink road_link;
  RoadProfiles road_profiles;

  RoadAttributes attributes;
  Lanes lanes;

  std::vector<TrafficSignalInformation> traffic_signals;
  std::vector<TrafficSignalReference> traffic_signal_references;

  std::vector<std::unique_ptr<GeometryAttributes>> geometry_attributes;
};

/////////////////////////////////////////////////////////////////

struct JunctionAttribues {
  int id;
  std::string name;

  JunctionAttribues() : id(-1) {}
};

struct JunctionConnectionAttributes {
  int id;
  int incoming_road;
  int connecting_road;
  std::string contact_point;

  JunctionConnectionAttributes()
      : id(-1), incoming_road(-1), connecting_road(-1) {}
};

struct JunctionLaneLink {
  int from;
  int to;

  JunctionLaneLink() : from(-1), to(-1) {}
};

struct JunctionConnection {
  JunctionConnectionAttributes attributes;
  std::vector<JunctionLaneLink> links;
};

struct Junction {
  JunctionAttribues attributes;
  std::vector<JunctionConnection> connections;
};

struct BoxComponent {
  double x_pos, y_pos, z_pos;
  double x_rot, y_rot, z_rot;
  double scale;
  BoxComponent()
      : x_pos(0.0),
        y_pos(0.0),
        z_pos(0.0),
        x_rot(0.0),
        y_rot(0.0),
        z_rot(0.0),
        scale(1.0) {}
};

struct TrafficLight {
  double x_pos, y_pos, z_pos;
  double x_rot, y_rot, z_rot;
  double scale;
  std::vector<BoxComponent> box_areas;

  TrafficLight()
      : x_pos(0.0),
        y_pos(0.0),
        z_pos(0.0),
        x_rot(0.0),
        y_rot(0.0),
        z_rot(0.0),
        scale(1.0) {}
};

struct TrafficLightGroup {
  std::vector<TrafficLight> traffic_lights;
  double red_time, yellow_time, green_time;
};

struct TrafficSign {
  double x_pos, y_pos, z_pos;
  double x_rot, y_rot, z_rot;
  double scale;
  int speed;
  std::vector<BoxComponent> box_areas;

  TrafficSign()
      : x_pos(0.0),
        y_pos(0.0),
        z_pos(0.0),
        x_rot(0.0),
        y_rot(0.0),
        z_rot(0.0),
        scale(1.0),
        speed(30) {}
};

/////////////////////////////////////////////////////////////////
namespace geom {
struct GeoLocation {
  double latitude{std::numeric_limits<double>::quiet_NaN()};
  double longitude{std::numeric_limits<double>::quiet_NaN()};
  double altitude{0.0};
  std::string projection;
};
}  // namespace geom

/////////////////////////////////////////////////////////////////
struct Lane {
  Id id{0u};
  LaneType type;
  Edge leftEdge;
  Edge rightEdge;
  std::vector<Id> successors;
  std::vector<Id> predecessors;
  std::vector<Id> overlaps;
  std::vector<ParametricSpeed> speed;
  std::vector<SignalReference> signalReferences;

  Id leftNeighbor{0u};
  Id rightNeighbor{0u};
  int index{0};
  int junction{-1};
};

struct Landmark {
  int id{-1};
  int type{-1};
  int subtype{-1};
  Point position{0., 0.};
  double orientation{0.};
};
using LaneMap = std::unordered_map<Id, Lane>;
using LandmarkMap = std::unordered_map<int, Landmark>;

struct OpenDriveData {
  geom::GeoLocation geoReference;
  std::vector<RoadInformation> roads;
  std::vector<Junction> junctions;
  std::vector<TrafficLightGroup> trafficlightgroups;
  std::vector<TrafficSign> trafficsigns;

  LaneMap laneMap;
  LandmarkMap landmarks;
  std::unordered_map<int, std::vector<Id>> intersectionLaneIds;
};
}  // namespace opendrive
