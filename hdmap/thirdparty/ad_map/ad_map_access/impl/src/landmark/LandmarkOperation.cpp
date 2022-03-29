// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/landmark/LandmarkOperation.hpp"
#include "ad/map/access/Logging.hpp"
#include "ad/map/access/Operation.hpp"
#include "ad/map/access/Store.hpp"
#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/point/Operation.hpp"

namespace ad {
namespace map {
namespace landmark {

////////////
// Getters

point::ENUHeading getENUHeading(Landmark const &landmark)
{
  return point::createENUHeading(point::createECEFHeading(landmark.position, landmark.orientation));
}

Landmark::ConstPtr getLandmarkPtr(LandmarkId const &id)
{
  auto landmarkPtr = access::getStore().getLandmarkPtr(id);

  if (!bool(landmarkPtr))
  {
    throw std::invalid_argument("ad::map::landmark::getLandmarkPtr: LandmarkId not found in store");
  }
  return landmarkPtr;
}

Landmark const &getLandmark(LandmarkId const &id)
{
  return *getLandmarkPtr(id);
}

landmark::LandmarkIdList getLandmarks()
{
  return access::getStore().getLandmarks();
}

ENULandmark getENULandmark(LandmarkId const &id)
{
  auto const landmarkPtr = getLandmarkPtr(id);
  ENULandmark landmark;
  landmark.id = landmarkPtr->id;
  landmark.type = landmarkPtr->type;
  landmark.position = point::toENU(landmarkPtr->position);
  landmark.trafficLightType = landmarkPtr->trafficLightType;
  landmark.heading = getENUHeading(*landmarkPtr);

  return landmark;
}

LandmarkIdList getVisibleLandmarks(lane::LaneId const &laneId)
{
  LandmarkIdList landmarks;
  auto const lanePtr = lane::getLanePtr(laneId);
  if (!bool(lanePtr))
  {
    throw std::invalid_argument("ad::map::landmark::getVisibleLandmarks: laneId not found in store");
  }
  return lanePtr->visibleLandmarks;
}

LandmarkIdList getVisibleLandmarks(LandmarkType const &landmarkType, lane::LaneId const &laneId)
{
  LandmarkIdList landmarks;
  auto const visibleLandmarkIds = getVisibleLandmarks(laneId);
  for (const auto &landmarkId : visibleLandmarkIds)
  {
    auto const landmarkPtr = getLandmarkPtr(landmarkId);
    if (landmarkPtr && landmarkPtr->type == landmarkType)
    {
      landmarks.push_back(landmarkPtr->id);
    }
  }

  return landmarks;
}

LandmarkId uniqueLandmarkId(point::GeoPoint const &geoPoint)
{
  LandmarkIdList landmarksIds;
  LandmarkId id;

  landmarksIds = access::getStore().getLandmarks();
  if (landmarksIds.size() == 0)
    throw std::invalid_argument("There is no landmarks in the map.");

  point::ECEFPoint ecefDstPoint = toECEF(geoPoint);
  physics::Distance minDistance = std::numeric_limits<physics::Distance>::max();
  for (auto const &landmarkId : landmarksIds)
  {
    auto const landmarkPtr = getLandmarkPtr(landmarkId);
    point::GeoPoint geoSrcPoint = toGeo(landmarkPtr->position);
    geoSrcPoint.altitude = geoPoint.altitude;
    point::ECEFPoint ecefSrcPoint = toECEF(geoSrcPoint);
    auto distance = point::distance(ecefSrcPoint, ecefDstPoint);

    if (landmarkPtr->boundingBox.isValid)
    {
      if (distance <= landmarkPtr->boundingBox.length && distance < minDistance)
      {
        minDistance = distance;
        id = landmarkPtr->id;
      }
    }
    else if (distance <= physics::Distance(10) && distance < minDistance)
    {
      minDistance = distance;
      id = landmarkPtr->id;
    }
  }

  if ((uint64_t)id == std::numeric_limits<uint64_t>::quiet_NaN())
    throw std::invalid_argument("Cannot find any lardmark given geo point.");

  return id;
}

} // namespace landmark
} // namespace map
} // namespace ad
