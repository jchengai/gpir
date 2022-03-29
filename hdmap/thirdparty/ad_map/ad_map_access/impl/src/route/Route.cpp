// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/route/Route.hpp"
#include "ad/map/lane/LaneOperation.hpp"

namespace ad {
namespace map {
namespace route {
namespace planning {

Route::Route(const RoutingParaPoint &start,
             const RoutingParaPoint &dest,
             physics::Distance const &maxDistance,
             physics::Duration const &maxDuration,
             Type const &routingType)
  : mStart(start)
  , mDest(dest)
  , mMaxDistance(maxDistance)
  , mMaxDuration(maxDuration)
  , mType(routingType)
  , mValid(false)
{
  if (mType == Type::INVALID)
  {
    throw std::runtime_error("type INVALID");
  }
}

bool Route::laneDirectionIsIgnored() const
{
  switch (mType)
  {
    case Type::SHORTEST_IGNORE_DIRECTION:
      return true;
    case Type::SHORTEST:
    default:
      return false;
  }
}

const Route::RawRoute &Route::getRawRoute(size_t const routeIndex) const
{
  static RawRoute const emptyRoute;
  if (mRawRoutes.size() > routeIndex)
  {
    return mRawRoutes[routeIndex];
  }
  return emptyRoute;
}

Route::BasicRoute Route::getBasicRoute(size_t const routeIndex) const
{
  auto rawRoute = getRawRoute(routeIndex);
  BasicRoute fr;
  for (size_t i = 0; i < rawRoute.paraPointList.size(); i++)
  {
    const point::ParaPoint &paraPoint = rawRoute.paraPointList[i];
    point::ParaPointList pps;
    pps.push_back(paraPoint);
    for (auto contactLocation : {lane::ContactLocation::LEFT, lane::ContactLocation::RIGHT})
    {
      for (lane::Lane::ConstPtr lane = lane::getLanePtr(paraPoint.laneId); lane;)
      {
        lane::LaneDirection const direction = lane->direction;
        lane::ContactLaneList const cl = getContactLanes(*lane, contactLocation);
        lane = nullptr;
        for (auto contact : cl)
        {
          lane::LaneId otherLaneId = contact.toLane;
          bool isPrev = false;
          if (i > 0)
          {
            isPrev = otherLaneId == rawRoute.paraPointList[i - 1].laneId;
          }
          bool isNext = false;
          if (i + 1 < rawRoute.paraPointList.size())
          {
            isNext = otherLaneId == rawRoute.paraPointList[i + 1].laneId;
          }
          if (!isNext && !isPrev)
          {
            lane::Lane::ConstPtr other_lane = lane::getLanePtr(otherLaneId);
            if ((direction == other_lane->direction) || laneDirectionIsIgnored())
            {
              point::ParaPoint ppt;
              ppt.laneId = otherLaneId;
              ppt.parametricOffset = paraPoint.parametricOffset;
              pps.push_back(ppt);
              lane = other_lane;
            }
          }
        }
      }
    }
    fr.push_back(pps);
  }
  return fr;
}

std::vector<Route::BasicRoute> Route::getBasicRoutes() const
{
  std::vector<BasicRoute> routeVector;
  routeVector.resize(mRawRoutes.size());
  for (size_t i = 0u; i < mRawRoutes.size(); ++i)
  {
    routeVector[i] = getBasicRoute(i);
  }
  return routeVector;
}

} // namespace planning
} // namespace route
} // namespace map
} // namespace ad
