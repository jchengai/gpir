// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "GeometryStore.hpp"
#include "ad/map/access/Factory.hpp"
#include "ad/map/access/Logging.hpp"
#include "ad/map/access/Operation.hpp"
#include "ad/map/access/Store.hpp"
#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/serialize/SerializeGeneratedTypes.hpp"

#include <algorithm>
#include <string>

namespace ad {
namespace map {
namespace access {

bool Store::save(serialize::ISerializer &serializer,
                 bool use_magic,
                 bool use_embedded_geometry,
                 bool use_geometry_store)
{
  if (serializer.isStoring())
  {
    use_magic_ = use_magic;
    use_embedded_geometry_ = use_embedded_geometry;
    use_geometry_store_ = use_geometry_store;
    return serialize(serializer);
  }
  else
  {
    access::getLogger()->error("Cannot save to read-only serializer.");
  }
  return false;
}

bool Store::load(serialize::ISerializer &serializer)
{
  bool ok = false;
  if (!serializer.isStoring())
  {
    ok = serialize(serializer);
  }
  else
  {
    access::getLogger()->error("Cannot load from to write-only serializer.");
  }
  return ok;
}

////////////////////////////////
// ISerializeable implementation

bool Store::isValid() const
{
  if (!access::isValid(meta_data_, false))
  {
    return false;
  }
  for (auto id_and_lane : lane_map_)
  {
    const lane::LaneId &lane_id = id_and_lane.first;
    const lane::Lane::Ptr &lane = id_and_lane.second;
    if (!lane::isValid(lane_id, false) || !lane || !lane::isValid(*lane, false))
    {
      return false;
    }
  }
  for (auto partition_id_and_ids : part_lane_map_)
  {
    const lane::LaneIdList &lane_ids = partition_id_and_ids.second;
    for (auto lane_id : lane_ids)
    {
      if (lane_map_.find(lane_id) == lane_map_.end())
      {
        return false;
      }
    }
  }
  for (auto partition_id_and_ids : part_landmark_map_)
  {
    const landmark::LandmarkIdList &landmark_ids = partition_id_and_ids.second;
    for (auto landmark_id : landmark_ids)
    {
      if (landmark_map_.find(landmark_id) == landmark_map_.end())
      {
        return false;
      }
    }
  }
  return true;
}

bool Store::serialize(serialize::ISerializer &serializer)
{
  bool old_magic = serializer.setUseMagic(true);
  bool old_use_embedded_points = serializer.setUseEmbeddedPoints(true);

  bool ok = serializer.serialize(serialize::SerializeableMagic::Store) && serializer.serialize(use_magic_)
    && serializer.serialize(use_embedded_geometry_) && serializer.serialize(use_geometry_store_);

  /*Todo the two lines below will be deleted after the map are generated again without use_zfp_*/
  bool use_zfp_ = false;
  ok = ok && serializer.serialize(use_zfp_);

  serializer.setUseMagic(use_magic_);
  serializer.setUseEmbeddedPoints(use_embedded_geometry_);
  ok = ok && doSerialize(serializer, meta_data_);
  ok = ok && serializer.serializeObjectPtrMap(lane_map_);
  ok = ok && serializer.serializeObjectPtrMap(landmark_map_);
  if (ok)
  {
    ok = serializer.serializeObjectVecMap(part_lane_map_);
    ok = serializer.serializeObjectVecMap(part_landmark_map_);
  }
  if (ok) // Todo will delete this after preparing new map without connector
  {
    if (serializer.isStoring())
    {
      ok = serializer.serializeEmptyObjectVec() && serializer.serializeEmptyObjectVec();
    }
    else
    {
      ok = serializer.serializeEmptyObjectVec() && serializer.serializeEmptyObjectVec();
    }
  }
  if (ok)
  {
    if (use_geometry_store_)
    {
      GeometryStore gs;
      if (serializer.isStoring())
      {
        ok = storeGeometry(gs);
      }
      if (ok)
      {
        ok = gs.serialize(serializer);
        if (ok && !serializer.isStoring())
        {
          if (use_embedded_geometry_)
          {
            ok = checkGeometry(gs);
          }
          else
          {
            ok = restoreGeometry(gs);
          }
        }
      }
    }
  }
  serializer.setUseMagic(old_magic);
  serializer.setUseEmbeddedPoints(old_use_embedded_points);
  return ok;
}

bool Store::storeGeometry(GeometryStore &gs)
{
  for (auto lane : lane_map_)
  {
    if (!gs.store(lane.second))
    {
      access::getLogger()->error("Store geometry failed for lane {}", lane.first);
      return false;
    }
  }
  return true;
}

bool Store::restoreGeometry(const GeometryStore &gs)
{
  for (auto lane : lane_map_)
  {
    if (!gs.restore(lane.second))
    {
      access::getLogger()->error("Restore geometry failed for lane {}", lane.first);
      return false;
    }
  }
  return true;
}

bool Store::checkGeometry(const GeometryStore &gs)
{
  for (auto lane : lane_map_)
  {
    if (!gs.check(lane.second))
    {
      access::getLogger()->error("Redundant geometry check failed!");
      return false;
    }
  }
  access::getLogger()->info("Redundant geometry check ok.");
  return true;
}

} // namespace access
} // namespace map
} // namespace ad
