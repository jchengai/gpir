// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <map>
#include <memory>

#include "ad/map/access/Types.hpp"
#include "ad/map/landmark/Types.hpp"
#include "ad/map/lane/Types.hpp"
#include "ad/map/serialize/ISerializer.hpp"

namespace ad {
namespace map {

namespace match {
class AdMapMatching;
}

namespace access {

class GeometryStore;

/**
 * @brief Autonomus Driving Map Store.
 */
class Store
{
  friend class Factory;
  friend class match::AdMapMatching;

public:                               // Derived types
  typedef std::shared_ptr<Store> Ptr; ///< Smart pointer to the Store.

  /**
   * @brief Constructor.
   *        Initializes empty Store.
   */
  Store();

  /**
   * @brief Copy constructor.
   */
  Store(Store const &other) = delete;

  /**
   * @brief Move constructor.
   */
  Store(Store &&other) = delete;

  /**
   * @brief Copy operator deleted.
   */
  Store &operator=(Store const &other) = delete;

  /**
   * @brief Move operator deleted.
   */
  Store &operator=(Store &&other) = delete;

  /**
   * @brief Destructor.
   *        Releases all resources.
   */
  virtual ~Store();

public: // Main Operations
  /**
   * @brief Save current AD Map Data Store.
   * @param[in] serializer Serializer to be used.
   * @param[in] use_magic          Use Magic Numbers for consistency during serialization.
   * @param[in] use_embedded_geometry  Save geometry together with objects.
   * @param[in] use_geometry_store Save geometry in separate section of file.
   * @return true if successful.
   */
  bool save(serialize::ISerializer &serializer,
            bool use_magic = true,
            bool use_embedded_geometry = true,
            bool use_geometry_store = false);

  /**
   * @brief Load data into the AD Map Data Store.
   * @param[in] serializer Serializer to be used.
   * @return true if successful.
   */
  bool load(serialize::ISerializer &serializer);

  /**
   * @returns true if there are no data in the store.
   */
  bool empty() const;

public: // Access Operations
  /**
   * @returns MetaData of the map.
   */
  MapMetaData const &getMetaData() const;

  /**
   * @returns Partitions currently present in the map.
   */
  PartitionIdList getPartitions() const;

  /**
   * @brief Method to be called to retrieve Lane from the Store.
   * @param[in] id Lane identifier.
   * @returns Lane with given identifier.
   *          Returned object may be empty if id is not valid or if Lane does not exists
   *          in the store.
   */
  lane::Lane::ConstPtr getLanePtr(lane::LaneId const &id) const;

  /**
   * @brief Method to be called to retrieve identifiers of all Lanes in the Store.
   * @returns Identifiers of all lanes in the store.
   */
  lane::LaneIdList getLanes() const;

  /**
   * @brief Retrieve identifiers of all Lanes belonging to specific partition.
   * @param[in] partition_id Partition identifier.
   * @returns Identifiers of all lanes in the store.
   */
  lane::LaneIdList getLanes(PartitionId const &partition_id) const;

  /**
   * @brief Method to be called to retrieve identifiers of filtered Lanes in the Store.
   * @param[in] type_filter    Type of the lane as string.
   * @param[in] is_hov         True if only lanes with HOV restriction shall be returned.
   * @returns Identifiers of all lanes in the store that satisfy given conditions.
   */
  lane::LaneIdList getLanes(std::string const &type_filter, bool is_hov) const;

  /**
   * @brief Method to be called to retrieve identifiers of filtered Lanes from partition.
   * @param[in] partition_id   Partition identifier.
   * @param[in] type_filter    Type of the lane as string.
   * @param[in] is_hov         True if only lanes with HOV restriction shall be returned.
   * @returns Identifiers of all lanes in the tile that satisfy given conditions.
   */
  lane::LaneIdList getLanes(PartitionId partition_id, std::string const &type_filter, bool is_hov) const;

  /**
   * @brief Retrieve identifiers of all Landmarks belonging to specific partition.
   * @param[in] partition_id Partition identifier.
   * @returns Identifiers of all lanes in the store.
   */
  landmark::LandmarkIdList getLandmarks(PartitionId partition_id) const;

  /**
   * @brief Method to be called to retrieve Landmark from the Store.
   * @param[in] id Landmark identifier.
   * @returns Landmark with given identifier.
   *          Returned object may be empty if id is not valid or if Landmark does not exists
   *          in the store.
   */
  landmark::Landmark::ConstPtr getLandmarkPtr(landmark::LandmarkId id) const;

  /**
   * @brief Method to be called to retrieve identifiers of all Landmarks in the Store.
   * @returns Identifiers of all landmarks in the store.
   */
  landmark::LandmarkIdList getLandmarks() const;

  /**
   * @brief Method to be called to remove all content belonging to specific partition and
   *        the partition itself.
   * @param[in] partition_id Partition identifier.
   */
  void removePartition(PartitionId partition_id);

  /**
   * @brief   Calculates cumulative length of the all lanes in the store.
   * @returns Cumulative length of all lanes in the store.
   */
  physics::Distance getCumulativeLaneLength() const;

  bool isValid() const;

  /*!
   * @returns The calculated bounding sphere of all lanes in the store.
   */
  point::BoundingSphere getBoundingSphere() const;

private:
  bool serialize(serialize::ISerializer &serializer);
  /**
   * @brief Store lane geometry to the Geometry Store.
   * @param[in] gs Geometry Store to be used.
   * @returns true if successful.
   */
  bool storeGeometry(GeometryStore &gs);

  /**
   * @brief Restore lane geometry from the Geometry Store.
   * @param[in] gs Geometry Store to be used.
   * @returns true if successful.
   */
  bool restoreGeometry(const GeometryStore &gs);

  /**
   * @brief Checks if lane geometry is same as one in the Geometry Store.
   * @param[in] gs Geometry Store to be used.
   * @returns true if successful.
   */
  bool checkGeometry(const GeometryStore &gs);

private:                       // Constructor
  bool use_magic_;             ///< Use Magic Numbers for consistency during serialization.
  bool use_embedded_geometry_; ///< Save geometry together with objects.
  bool use_geometry_store_;    ///< Save geometry in separate section of file.

  typedef std::map<lane::LaneId, lane::Lane::Ptr> LaneMap;                     ///< Map LaneId/Lane.
  typedef std::map<landmark::LandmarkId, landmark::Landmark::Ptr> LandmarkMap; ///< Map LandmarkId/Landmark.
  typedef std::map<PartitionId, lane::LaneIdList> PartLaneMap;                 ///< Map PartitionId/LaneIdList.
  typedef std::map<PartitionId, landmark::LandmarkIdList> PartLandmarkMap;     ///< Map PartitionId/LandmarkIdList.

  MapMetaData meta_data_;             ///< General map meta data
  LaneMap lane_map_;                  ///< All Lane-s currently in the store.
  LandmarkMap landmark_map_;          ///< All Landmark-s currently in the store.
  PartLaneMap part_lane_map_;         ///< Lane identifiers belonging to the tile.
  PartLandmarkMap part_landmark_map_; ///< Landmark identifiers belonging to the tile.
};

} // namespace access
} // namespace map
} // namespace ad
