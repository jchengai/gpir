// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "GeometryStore.hpp"
#include <memory>
#include "ad/map/access/Logging.hpp"
#include "ad/map/access/Store.hpp"
#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/point/GeometryOperation.hpp"
#include "ad/map/serialize/SerializeGeneratedTypes.hpp"

namespace ad {
namespace map {
namespace access {

GeometryStore::GeometryStore()
{
  store_ = nullptr;
  points3d_ = 0;
  capacity3d_ = 0;
}

GeometryStore::~GeometryStore()
{
  destroy();
}

/////////////
// Operations

bool GeometryStore::store(lane::Lane::ConstPtr lane)
{
  if (lane)
  {
    lane::LaneId id = lane->id;
    auto it = lane_items_.find(id);
    if (it == lane_items_.end())
    {
      uint32_t offset_left = 0, size_left = 0;
      if (store(lane, lane::ContactLocation::LEFT, offset_left, size_left))
      {
        uint32_t offset_right = 0, size_right = 0;
        if (store(lane, lane::ContactLocation::RIGHT, offset_right, size_right))
        {
          GeometryStoreItem item;
          item.leftEdgeOffset = offset_left;
          item.leftEdgePoints = size_left;
          item.rightEdgeOffset = offset_right;
          item.rightEdgePoints = size_right;
          lane_items_[id] = item;
          return true;
        }
      }
    }
    else
    {
      access::getLogger()->error("GeometryStore: Lane already in Store?! {}", id);
      throw std::runtime_error("GeometryStore: Lane already in Store?! ");
    }
  }
  else
  {
    throw std::runtime_error("GeometryStore: Lane invalid");
  }
  return false;
}

bool GeometryStore::restore(lane::Lane::Ptr lane) const
{
  if (lane)
  {
    lane::LaneId id = lane->id;
    auto it = lane_items_.find(id);
    if (it != lane_items_.end())
    {
      const GeometryStoreItem &item = it->second;
      point::ECEFEdge left;
      if (restore(left, item.leftEdgeOffset, item.leftEdgePoints))
      {
        point::ECEFEdge right;
        if (restore(right, item.rightEdgeOffset, item.rightEdgePoints))
        {
          lane->edgeLeft = point::createGeometry(left, false);
          lane->edgeRight = point::createGeometry(right, false);
          return true;
        }
        else
        {
          access::getLogger()->error("GeometryStore: Lane right edge not in Store?! {}", id);
        }
      }
      else
      {
        access::getLogger()->error("GeometryStore: Lane left edge not in Store?! {}", id);
      }
    }
    else
    {
      access::getLogger()->error("GeometryStore: Lane not in Store?! {}", id);
    }
  }
  else
  {
    throw std::runtime_error("GeometryStore: Lane invalid");
  }
  return false;
}

bool GeometryStore::check(lane::Lane::ConstPtr lane) const
{
  if (lane)
  {
    lane::LaneId id = lane->id;
    auto it = lane_items_.find(id);
    if (it != lane_items_.end())
    {
      const GeometryStoreItem &item = it->second;
      point::ECEFEdge left;
      if (restore(left, item.leftEdgeOffset, item.leftEdgePoints))
      {
        point::ECEFEdge right;
        if (restore(right, item.rightEdgeOffset, item.rightEdgePoints))
        {
          if (lane->edgeLeft.ecefEdge == left && lane->edgeRight.ecefEdge == right)
          {
            return true;
          }
          else
          {
            access::getLogger()->error("GeometryStore: Lane geometry mismatch?! {}", id);
          }
        }
        else
        {
          access::getLogger()->error("GeometryStore: Lane right edge not in Store?! {}", id);
        }
      }
      else
      {
        access::getLogger()->error("GeometryStore: Lane left edge not in Store?! {}", id);
      }
    }
    else
    {
      access::getLogger()->error("GeometryStore: Lane not in Store?! {}", id);
    }
  }
  else
  {
    throw std::runtime_error("GeometryStore: Lane invalid");
  }
  return false;
}

///////////////
// Aux Methods

bool GeometryStore::store(lane::Lane::ConstPtr lane, lane::ContactLocation location, uint32_t &offs3d, uint32_t &size)
{
  if ((location != lane::ContactLocation::LEFT) && (location != lane::ContactLocation::RIGHT))
  {
    throw std::runtime_error("Location must be LEFT or RIGHT");
  }
  const point::ECEFEdge &ecef
    = (location == lane::ContactLocation::LEFT) ? lane->edgeLeft.ecefEdge : lane->edgeRight.ecefEdge;
  size = static_cast<uint32_t>(ecef.size());
  lane::ContactLaneList contact_lanes = lane::getContactLanes(*lane, location);
  for (auto contact_lane : contact_lanes)
  {
    lane::LaneId contact_lane_id = contact_lane.toLane;
    auto it = lane_items_.find(contact_lane_id);
    if (it != lane_items_.end())
    {
      lane::Lane::ConstPtr clane = lane::getLanePtr(contact_lane_id);
      if (clane)
      {
        const point::ECEFEdge &ecef1
          = (location == lane::ContactLocation::LEFT) ? clane->edgeRight.ecefEdge : clane->edgeLeft.ecefEdge;
        if (ecef == ecef1)
        {
          offs3d = (location == lane::ContactLocation::LEFT) ? it->second.rightEdgeOffset : it->second.leftEdgeOffset;
          return true;
        }
      }
    }
  }
  return store(ecef, offs3d);
}

bool GeometryStore::store(const point::ECEFEdge &ecef, uint32_t &offset3d)
{
  while (points3d_ + ecef.size() >= capacity3d_)
  {
    if (!expand())
    {
      return false;
    }
  }
  offset3d = points3d_;
  for (auto pt : ecef)
  {
    uint32_t index = (points3d_++) * 3;
    store_[index++] = static_cast<double>(pt.x);
    store_[index++] = static_cast<double>(pt.y);
    store_[index++] = static_cast<double>(pt.z);
  }
  return true;
}

bool GeometryStore::restore(point::ECEFEdge &ecef, uint32_t offset3d, uint32_t points3d) const
{
  if (!ecef.empty())
  {
    throw std::runtime_error("ecef not empty");
  }
  if (offset3d + points3d <= capacity3d_)
  {
    for (uint32_t index = offset3d * 3; points3d != 0; points3d--)
    {
      point::ECEFPoint pt;
      pt.x = point::ECEFCoordinate(store_[index++]);
      pt.y = point::ECEFCoordinate(store_[index++]);
      pt.z = point::ECEFCoordinate(store_[index++]);
      ecef.push_back(pt);
    }
    return true;
  }
  else
  {
    return false;
  }
}

//////////////
// Aux Methods

void GeometryStore::destroy()
{
  if (store_ != nullptr)
  {
    free(store_);
    store_ = nullptr;
    points3d_ = 0;
    capacity3d_ = 0;
    access::getLogger()->debug("GeometryStore: Destroyed.");
  }
}

bool GeometryStore::expand()
{
  if (store_ == nullptr)
  {
    return create(SIZE_INCREMENT);
  }
  else
  {
    size_t bytes = (capacity3d_ + SIZE_INCREMENT) * 3 * sizeof(double);
    double *store = static_cast<double *>(std::realloc(store_, bytes));
    if (store != nullptr)
    {
      store_ = store;
      capacity3d_ += SIZE_INCREMENT;
      return true;
    }
    else
    {
      access::getLogger()->error("GeometryStore: Cannot expand to {} bytes.", bytes);
      return false;
    }
  }
}

bool GeometryStore::create(uint32_t capacity3d)
{
  destroy();
  size_t bytes = capacity3d * 3 * sizeof(double);
  store_ = static_cast<double *>(std::malloc(bytes));
  if (store_ != nullptr)
  {
    points3d_ = 0;
    capacity3d_ = capacity3d;
    return true;
  }
  else
  {
    access::getLogger()->error("GeometryStore: Cannot allocate {} bytes.", bytes);
    return false;
  }
}

bool GeometryStore::serialize(serialize::ISerializer &serializer)
{
  bool ok = serializer.serialize(serialize::SerializeableMagic::GeometryStore)
    && serializer.serializeObjectMap(lane_items_) && serializer.serialize(points3d_);

  /*Todo the two lines below will be deleted after the map are generated again without use_zfp_*/
  bool use_zfp_ = false;
  ok = ok && serializer.serialize(use_zfp_);

  if (ok)
  {
    if (!serializer.isStoring())
    {
      if (create(points3d_))
      {
        points3d_ = capacity3d_;
      }
      else
      {
        return false;
      }
      ok = serializer.read(store_, points3d_ * 3 * sizeof(double));
    }
    else
    {
      ok = serializer.write(store_, points3d_ * 3 * sizeof(double));
    }
  }
  return ok;
}

} // namespace access
} // namespace map
} // namespace ad
