// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/access/Store.hpp"
#include "ad/map/access/Logging.hpp"

#include <algorithm>
#include <cstring>
#include <string>
#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/point/BoundingSphereOperation.hpp"

namespace ad {
namespace map {
namespace access {

Store::Store()
{
  use_magic_ = false;
  use_embedded_geometry_ = true;
  use_geometry_store_ = false;
}

Store::~Store()
{
}

bool Store::empty() const
{
  return lane_map_.empty() && landmark_map_.empty();
}

////////////////////
// Access Operations
MapMetaData const &Store::getMetaData() const
{
  return meta_data_;
}

PartitionIdList Store::getPartitions() const
{
  PartitionIdList part_ids;
  for (auto partition_id_and_ids : part_lane_map_)
  {
    part_ids.push_back(partition_id_and_ids.first);
  }
  for (auto partition_id_and_ids : part_landmark_map_)
  {
    if (std::find(part_ids.begin(), part_ids.end(), partition_id_and_ids.first) == part_ids.end())
    {
      part_ids.push_back(partition_id_and_ids.first);
    }
  }
  return part_ids;
}

lane::Lane::ConstPtr Store::getLanePtr(lane::LaneId const &id) const
{
  lane::Lane::ConstPtr lane_ptr;
  auto id_and_lane = lane_map_.find(id);
  if (id_and_lane != lane_map_.end())
  {
    lane_ptr = id_and_lane->second;
  }
  else
  {
    access::getLogger()->warn("Lane not in the Store. ID: {}", id);
  }
  return lane_ptr;
}

lane::LaneIdList Store::getLanes() const
{
  lane::LaneIdList ids;
  for (auto partition_id_and_ids : part_lane_map_)
  {
    const lane::LaneIdList &tids = partition_id_and_ids.second;
    ids.insert(ids.end(), tids.begin(), tids.end());
  }
  return ids;
}

lane::LaneIdList Store::getLanes(PartitionId const &partition_id) const
{
  auto partition_and_ids = part_lane_map_.find(partition_id);
  if (partition_and_ids != part_lane_map_.end())
  {
    return partition_and_ids->second;
  }
  return lane::LaneIdList();
}

lane::LaneIdList Store::getLanes(std::string const &type_filter, bool is_hov) const
{
  lane::LaneIdList ids;
  for (auto id_and_lane : lane_map_)
  {
    lane::Lane::Ptr lane = id_and_lane.second;
    if (lane && lane::satisfiesFilter(*lane, type_filter, is_hov))
    {
      ids.push_back(id_and_lane.first);
    }
  }
  return ids;
}

lane::LaneIdList Store::getLanes(PartitionId partition_id, std::string const &type_filter, bool is_hov) const
{
  lane::LaneIdList ids;
  auto partition_and_ids = part_lane_map_.find(partition_id);
  if (partition_and_ids != part_lane_map_.end())
  {
    for (auto lane_id : partition_and_ids->second)
    {
      auto findResult = lane_map_.find(lane_id);
      if (findResult != lane_map_.end())
      {
        if (lane::satisfiesFilter(*findResult->second, type_filter, is_hov))
        {
          ids.push_back(lane_id);
        }
      }
    }
  }
  return ids;
}

landmark::LandmarkIdList Store::getLandmarks(PartitionId partition_id) const
{
  auto partition_and_landmark = part_landmark_map_.find(partition_id);
  if (partition_and_landmark != part_landmark_map_.end())
  {
    return partition_and_landmark->second;
  }
  return landmark::LandmarkIdList();
}

landmark::Landmark::ConstPtr Store::getLandmarkPtr(landmark::LandmarkId id) const
{
  landmark::Landmark::ConstPtr landmark_ptr;
  auto id_and_landmark = landmark_map_.find(id);
  if (id_and_landmark != landmark_map_.end())
  {
    landmark_ptr = id_and_landmark->second;
  }
  else
  {
    access::getLogger()->warn("Landmark is not in the Store: {}", id);
  }
  return landmark_ptr;
}

landmark::LandmarkIdList Store::getLandmarks() const
{
  landmark::LandmarkIdList ids;
  for (auto id_and_landmark : landmark_map_)
  {
    ids.push_back(id_and_landmark.first);
  }
  return ids;
}

void Store::removePartition(PartitionId partition_id)
{
  {
    // remove lanes
    auto findLanesResult = part_lane_map_.find(partition_id);
    if (findLanesResult != part_lane_map_.end())
    {
      for (auto lane_id : findLanesResult->second)
      {
        lane_map_.erase(lane_id);
      }
      part_lane_map_.erase(findLanesResult);
    }
  }

  {
    // remove landmarks
    auto findLandmarkResult = part_landmark_map_.find(partition_id);
    if (findLandmarkResult != part_landmark_map_.end())
    {
      for (auto landmark_id : findLandmarkResult->second)
      {
        landmark_map_.erase(landmark_id);
      }
      part_landmark_map_.erase(findLandmarkResult);
    }
  }
}

/////////////
// Statistics

physics::Distance Store::getCumulativeLaneLength() const
{
  physics::Distance total(0);
  lane::LaneIdList ids = getLanes();
  for (auto id : ids)
  {
    lane::Lane::ConstPtr lane = getLanePtr(id);
    if (lane)
    {
      total += lane->length;
    }
  }
  return total;
}

point::BoundingSphere Store::getBoundingSphere() const
{
  point::BoundingSphere boundingSphere;
  if (lane_map_.empty())
  {
    return boundingSphere;
  }
  boundingSphere = lane_map_.begin()->second->boundingSphere;
  for (auto id_and_lane : lane_map_)
  {
    boundingSphere = boundingSphere + id_and_lane.second->boundingSphere;
  }
  return boundingSphere;
}

} // namespace access
} // namespace map
} // namespace ad
