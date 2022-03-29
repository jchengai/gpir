// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/serialize/SerializeGeneratedLaneTypes.hpp"
#include "ad/map/point/BoundingSphereOperation.hpp"
#include "ad/map/point/ECEFOperation.hpp"
#include "ad/map/serialize/SerializeGeneratedTypes.hpp"

namespace ad {
namespace map {
namespace serialize {

bool doSerialize(ISerializer &serializer, lane::Lane &lane)
{
  bool ok = serializer.serialize(SerializeableMagic::Lane) && doSerialize(serializer, lane.id)
    && serializer.serialize(lane.type) && serializer.serialize(lane.direction)
    && doSerialize(serializer, lane.restrictions) && doSerialize(serializer, lane.length)
    && doSerialize(serializer, lane.lengthRange) && doSerialize(serializer, lane.width)
    && doSerialize(serializer, lane.widthRange) && serializer.serializeObjectVector(lane.speedLimits)
    // @toDo: version 0.4 compat: keep Edge in for backward compat
    && serializer.serialize(SerializeableMagic::Edge) && doSerialize(serializer, lane.edgeLeft)
    // @toDo: version 0.4 compat: keep Edge in for backward compat
    && serializer.serialize(SerializeableMagic::Edge) && doSerialize(serializer, lane.edgeRight)
    && serializer.serializeObjectVector(lane.contactLanes) && serializer.serialize(SerializeableMagic::ComplianceVer)
    && serializer.serialize(lane.complianceVersion) && doSerialize(serializer, lane.boundingSphere)
    && serializer.serializeObjectVector(lane.visibleLandmarks);

  if (ok)
  {
    //@toDO: version 0.4 compat: cope with empty bounding sphere in old files
    if ((lane.boundingSphere.center == point::createECEFPoint(0., 0., 0.))
        || (lane.boundingSphere.radius == physics::Distance(0.)))
    {
      lane.boundingSphere = point::calcBoundingSphere(lane.edgeLeft, lane.edgeRight);
    }
  }
  return ok;
}

bool doSerialize(ISerializer &serializer, lane::ContactLane &contactLane)
{
  return serializer.serialize(SerializeableMagic::ContactLane) && doSerialize(serializer, contactLane.toLane)
    && serializer.serialize(contactLane.location) && serializer.serializeVector(contactLane.types)
    && doSerialize(serializer, contactLane.restrictions) && doSerialize(serializer, contactLane.trafficLightId);
}

} // namespace serialize
} // namespace map
} // namespace ad
