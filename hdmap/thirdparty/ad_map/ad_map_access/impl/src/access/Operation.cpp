// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/access/Operation.hpp"

#include <boost/filesystem/path.hpp>

#include "AdMapAccess.hpp"
#include "ad/map/config/MapConfigFileHandler.hpp"
#include "ad/map/opendrive/AdMapFactory.hpp"
#include "ad/map/point/Operation.hpp"
#include "ad/map/point/Transform.hpp"
#include "ad/map/serialize/SerializerFileCRC32.hpp"

namespace ad {
namespace map {
namespace access {

void setENUReferencePoint(point::GeoPoint const &point) {
  auto coordinateTransform = getCoordinateTransform();
  if (!coordinateTransform->isENUValid() ||
      (coordinateTransform->getENUReferencePoint() != point)) {
    coordinateTransform->setENUReferencePoint(point);
  }
}

std::shared_ptr<point::CoordinateTransform> getCoordinateTransform() {
  // coordinate transform (at least without ENURefPoint) can actually used
  // before initialization therefore, return the transform without
  // initialization check
  return AdMapAccess::getAdMapAccessInstance().mCoordinateTransform;
}

point::GeoPoint getENUReferencePoint() {
  return AdMapAccess::getAdMapAccessInstance()
      .mCoordinateTransform->getENUReferencePoint();
}

bool isENUReferencePointSet() {
  return AdMapAccess::getAdMapAccessInstance()
      .mCoordinateTransform->isENUValid();
}

std::vector<config::PointOfInterest> getPointsOfInterest(
    point::GeoPoint const &geoPoint, physics::Distance const &radius) {
  std::vector<config::PointOfInterest> resultVector;
  point::ECEFPoint const geoPointEcef = point::toECEF(geoPoint);
  for (auto const &poi : AdMapAccess::getInitializedInstance()
                             .mConfigFileHandler.pointsOfInterest()) {
    point::ECEFPoint const poiPointEcef = point::toECEF(poi.geoPoint);
    physics::Distance const poiDistance =
        point::distance(poiPointEcef, geoPointEcef);
    if (poiDistance <= radius) {
      resultVector.push_back(poi);
    }
  }
  return resultVector;
}

std::vector<config::PointOfInterest> const &getPointsOfInterest() {
  return AdMapAccess::getInitializedInstance()
      .mConfigFileHandler.pointsOfInterest();
}

bool getPointOfInterest(std::string const &name, config::PointOfInterest &poi) {
  for (auto pointOfInterest : getPointsOfInterest()) {
    if (pointOfInterest.name == name) {
      poi = pointOfInterest;
      return true;
    }
  }
  return false;
}

bool init(std::string const &configFileName) {
  // initialization has to be performed without initialization check
  return AdMapAccess::getAdMapAccessInstance().initialize(configFileName);
}

bool initFromOpenDriveContent(
    std::string const &openDriveContent, double const overlapMargin,
    intersection::IntersectionType const defaultIntersectionType,
    landmark::TrafficLightType const defaultTrafficLightType) {
  return AdMapAccess::getAdMapAccessInstance().initializeFromOpenDriveContent(
      openDriveContent, overlapMargin, defaultIntersectionType,
      defaultTrafficLightType);
}

bool init(Store::Ptr store) {
  // initialization has to be performed without initialization check
  return AdMapAccess::getAdMapAccessInstance().initialize(store);
}

void cleanup() {
  // initialization has to be performed without initialization check
  AdMapAccess::getAdMapAccessInstance().reset();
}

bool isLeftHandedTraffic() {
  return getStore().getMetaData().trafficType == TrafficType::LEFT_HAND_TRAFFIC;
}

bool isRightHandedTraffic() {
  return getStore().getMetaData().trafficType ==
         TrafficType::RIGHT_HAND_TRAFFIC;
}

Store &getStore() { return *AdMapAccess::getInitializedInstance().mStore; }

}  // namespace access
}  // namespace map
}  // namespace ad
