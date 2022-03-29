// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2019-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "DataTypeConversion.hpp"

#include "ad/map/point/Operation.hpp"

namespace ad {
namespace map {
namespace opendrive {

landmark::LandmarkType toLandmarkType(int type) {
  // as defined on the OpenDRIVEv1.4 spec chapter 6.11 Signal Types
  // and on the OpenDRIVEv1.5 spec chapter 6.11 Signal Types
  switch (type) {
    case 1000001:
    case 1000002:
    case 1000007:
    case 1000008:
    case 1000009:
    case 1000010:
    case 1000011:
    case 1000013:
    case 1000014:
    case 1000015:
    case 1000016:
    case 1000017:
    case 1000018:
    case 1000019:
    case 1000020:
      return landmark::LandmarkType::TRAFFIC_LIGHT;
    case 1000003:  // pedestrian crossing
    case 1000004:  // bicycle crossing
      return landmark::LandmarkType::OTHER;
    default:
      break;
  }

  if ((type > 100) && (type <= 631))  // as defined by the StVO signal types
  {
    return landmark::LandmarkType::TRAFFIC_SIGN;
  } else {
    return landmark::LandmarkType::UNKNOWN;
  }
}

landmark::TrafficSignType toTrafficSignType(int type, int subtype) {
  // as defined by the StVO signal type
  switch (type) {
    case 101:
      return landmark::TrafficSignType::DANGER;
    case 102:
    case 103:
    case 105:
    case 108:
    case 110:
    case 112:
    case 114:
      return landmark::TrafficSignType::DANGER;
    case 120:
    case 121:
      return landmark::TrafficSignType::LANES_MERGING;
    case 133:
      return landmark::TrafficSignType::CAUTION_PEDESTRIAN;
    case 138:
      return landmark::TrafficSignType::CAUTION_BICYCLE;
    case 205:
      return landmark::TrafficSignType::YIELD;
    case 206:
      return landmark::TrafficSignType::STOP;
    case 215:
      return landmark::TrafficSignType::ROUNDABOUT;
    case 222:
      return landmark::TrafficSignType::PASS_RIGHT;
    case 250:
      return landmark::TrafficSignType::ACCESS_FORBIDDEN;
    case 251:
      return landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES;
    case 253:
      return landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS;
    case 254:
      return landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE;
    case 263:
      return landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT;
    case 264:
      return landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH;
    case 265:
      return landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT;
    case 267:
      return landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR;
    case 274:
      if (subtype == -1) {
        return landmark::TrafficSignType::MAX_SPEED;
      } else if (subtype == 1) {
        return landmark::TrafficSignType::SPEED_ZONE_30_BEGIN;
      } else if (subtype == 2) {
        return landmark::TrafficSignType::SPEED_ZONE_30_END;
      } else {
        return landmark::TrafficSignType::MAX_SPEED;
      }
    case 301:
      return landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION;
    case 306:
      return landmark::TrafficSignType::PRIORITY_WAY;
    case 310:
      return landmark::TrafficSignType::CITY_BEGIN;
    case 311:
      return landmark::TrafficSignType::CITY_END;
    case 331:
      return landmark::TrafficSignType::MOTORVEHICLE_BEGIN;
    default:
      return landmark::TrafficSignType::UNKNOWN;
  }
}

lane::ContactType toContactType(int type) {
  switch (type) {
    case 205:
      return lane::ContactType::YIELD;
    case 206:
      return lane::ContactType::STOP;
    case 301:
      return lane::ContactType::RIGHT_OF_WAY;  // next intersection only
    case 306:
      return lane::ContactType::RIGHT_OF_WAY;
    case 1000001:
    case 1000008:
    case 1000009:
    case 1000010:
    case 1000011:
    case 1000012:
      return lane::ContactType::TRAFFIC_LIGHT;
    default:
      return lane::ContactType::UNKNOWN;
  }
}

lane::ContactLocation toContactLocation(
    ::opendrive::SignalReference const &signalReference,
    bool const &isJunction) {
  if (isJunction) {
    if (signalReference.parametricPosition >= 0.5) {
      return lane::ContactLocation::SUCCESSOR;
    } else {
      return lane::ContactLocation::PREDECESSOR;
    }
  } else {
    if (signalReference.inLaneOrientation) {
      return lane::ContactLocation::SUCCESSOR;
    } else {
      return lane::ContactLocation::PREDECESSOR;
    }
  }
}

landmark::TrafficLightType toTrafficLightType(int type, int subtype) {
  switch (type) {
    case 1000001:
      return landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN;
    case 1000002:
      return landmark::TrafficLightType::PEDESTRIAN_RED_GREEN;
    case 1000007:
      return landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN;
    case 1000008:
      return landmark::TrafficLightType::UNKNOWN;
    case 1000009:
      return landmark::TrafficLightType::SOLID_RED_YELLOW;
    case 1000010:
      return landmark::TrafficLightType::UNKNOWN;
    case 1000011:
      if (subtype == 10) {
        return landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN;
      } else if (subtype == 20) {
        return landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN;
      } else if (subtype == 30) {
        return landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN;
      } else if (subtype == 40) {
        return landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN;
      } else if (subtype == 50) {
        return landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN;
      } else {
        return landmark::TrafficLightType::UNKNOWN;
      }
    case 1000012:
      return landmark::TrafficLightType::UNKNOWN;
    case 1000013:
      return landmark::TrafficLightType::BIKE_RED_GREEN;
    case 1000014:
      return landmark::TrafficLightType::UNKNOWN;
    case 1000015:
      return landmark::TrafficLightType::UNKNOWN;
    default:
      return landmark::TrafficLightType::UNKNOWN;
  }
}

lane::LaneType toLaneType(::opendrive::LaneType const &laneType) {
  switch (laneType) {
    case ::opendrive::LaneType::Driving:
    case ::opendrive::LaneType::Bidirectional:
      return lane::LaneType::NORMAL;
    case ::opendrive::LaneType::Shoulder:
    case ::opendrive::LaneType::Border:
    case ::opendrive::LaneType::Parking:
      return lane::LaneType::SHOULDER;
    case ::opendrive::LaneType::Biking:
      return lane::LaneType::BIKE;
    case ::opendrive::LaneType::Sidewalk:
      return lane::LaneType::PEDESTRIAN;
    case ::opendrive::LaneType::Stop:
      return lane::LaneType::EMERGENCY;
    case ::opendrive::LaneType::Entry:
    case ::opendrive::LaneType::Exit:
    case ::opendrive::LaneType::OffRamp:
    case ::opendrive::LaneType::OnRamp:
      return lane::LaneType::TURN;
    case ::opendrive::LaneType::Restricted:
    case ::opendrive::LaneType::Median:
    case ::opendrive::LaneType::Special1:
    case ::opendrive::LaneType::Special2:
    case ::opendrive::LaneType::Special3:
    case ::opendrive::LaneType::RoadWorks:
    case ::opendrive::LaneType::Tram:
    case ::opendrive::LaneType::Rail:
    case ::opendrive::LaneType::None:
      return lane::LaneType::UNKNOWN;
    default:
      return lane::LaneType::INVALID;
  }
}

lane::LaneDirection toLaneDirection(::opendrive::Lane const &lane,
                                    bool rightHandTraffic) {
  lane::LaneDirection direction;

  // First determine the lane direction based on the position of the lane (right
  // hand traffic)
  if (lane.index <= 0) {
    if (rightHandTraffic) {
      direction = lane::LaneDirection::POSITIVE;
    } else {
      direction = lane::LaneDirection::NEGATIVE;
    }
  } else {
    if (rightHandTraffic) {
      direction = lane::LaneDirection::NEGATIVE;
    } else {
      direction = lane::LaneDirection::POSITIVE;
    }
  }

  // Then based on the lane type adjust the lane direction as needed
  switch (lane.type) {
    case ::opendrive::LaneType::Bidirectional:
      return lane::LaneDirection::BIDIRECTIONAL;
    case ::opendrive::LaneType::Driving:
    case ::opendrive::LaneType::Biking:
    case ::opendrive::LaneType::Stop:
    case ::opendrive::LaneType::Entry:
    case ::opendrive::LaneType::Exit:
    case ::opendrive::LaneType::OffRamp:
    case ::opendrive::LaneType::OnRamp:
    case ::opendrive::LaneType::Restricted:
    case ::opendrive::LaneType::Median:
    case ::opendrive::LaneType::Special1:
    case ::opendrive::LaneType::Special2:
    case ::opendrive::LaneType::Special3:
      return direction;
    case ::opendrive::LaneType::None:
    case ::opendrive::LaneType::Shoulder:
    case ::opendrive::LaneType::Border:
    case ::opendrive::LaneType::Sidewalk:
    case ::opendrive::LaneType::Parking:
      return lane::LaneDirection::NONE;
    case ::opendrive::LaneType::RoadWorks:
    case ::opendrive::LaneType::Tram:
    case ::opendrive::LaneType::Rail:
      return lane::LaneDirection::UNKNOWN;
    default:
      return lane::LaneDirection::INVALID;
  }
}

point::GeoPoint toGeo(::opendrive::Point const &point) {
  return point::createGeoPoint(point::Longitude(point.x),
                               point::Latitude(point.y), point::Altitude(0.));
}

point::ECEFPoint toECEF(::opendrive::Point const &point) {
  // points expected to be geo points
  return point::toECEF(toGeo(point));
}

point::Geometry toGeometry(std::vector<::opendrive::Point> edgePoints) {
  point::ECEFEdge ecefEdge;
  for (auto const &point : edgePoints) {
    ecefEdge.push_back(toECEF(point));
  }

  return point::createGeometry(ecefEdge, false);
}

}  // namespace opendrive
}  // namespace map
}  // namespace ad
