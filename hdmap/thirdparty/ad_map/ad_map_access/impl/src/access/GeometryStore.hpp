// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <map>
#include "ad/map/access/GeometryStoreItem.hpp"
#include "ad/map/lane/Lane.hpp"
#include "ad/map/serialize/ISerializer.hpp"

namespace ad {
namespace map {
namespace access {

/**
 * @brief Geometries container for serialization.
 */
class GeometryStore
{
public: // Constructor/Destructor
  /**
   * @brief Default constructor.
   *        Creates empty store.
   */
  explicit GeometryStore();

  /**
   * @brief Constructor.
   *        Releases all resources.
   */
  ~GeometryStore();

public: // Operations
  /**
   * @brief Stores Lane Edge geometry in to the store.
   * @param[in] lane Lane to be stored.
   * @returns true if successful.
   */
  bool store(lane::Lane::ConstPtr lane);

  /**
   * @brief Restores Lane Edge geometry from the store.
   * @param[in] lane Lane to be restored.
   * @returns true if successful.
   */
  bool restore(lane::Lane::Ptr lane) const;

  /**
   * @brief Check if Lane Edge geometry is same as one i the store.
   * @param[in] lane Lane to be restored.
   * @returns true if successful.
   */
  bool check(lane::Lane::ConstPtr lane) const;

private: // Aux Methods
  /**
   * @brief Stores vector of ECEFEdge to the store.
   * @param[in] ecef Vector to be stored.
   * @param[out] offset3d Offset where first point is stored.
   * @returns true if successful.
   */
  bool store(const point::ECEFEdge &ecef, uint32_t &offset3d);

  /**
   * @brief Stores the geometry, but first checks if geometry is already in a store.
   * @param[in] lane Lane which is to be stored.
   * @param[in] location Contact location of lane that may have same geometry.
   * @param[out] offset3d If successful, offset where first point is stored.
   * @return true if successful.
   */
  bool store(lane::Lane::ConstPtr lane, lane::ContactLocation location, uint32_t &offset3d, uint32_t &size);

  /**
   * @brief Restores vector of ECEFEdge to the store.
   * @param[in] ecef Vector to be restored.
   * @param[in] offset3d Offset where first point is stored.
   * @param[in] points3d Number of 3d points to be retrieved.
   * @returns true if successful.
   */
  bool restore(point::ECEFEdge &ecef, uint32_t offset3d, uint32_t points3d) const;

  /**
   * @brief Destroys the store.
   */
  void destroy();

  /**
   * @brief Expands the store.
   * @returns true if successful.
   */
  bool expand();

  /**
   * @brief Creates the store of the specified size.
   * @param[in] capacity3d Capacity of the store.
   * @returns true if successful.
   */
  bool create(uint32_t capacity3d);

public:
  bool serialize(serialize::ISerializer &serializer);

private:                                           // Constants
  static constexpr uint32_t SIZE_INCREMENT = 1024; ///< Number of points to be added each time
                                                   ///< store_ needs to be expaned.

private: // Data Members
  /**
   * @brief Copy constructor deleted.
   */
  GeometryStore(GeometryStore const &other) = delete;

  /**
   * @brief Move constructor deleted.
   */
  GeometryStore(GeometryStore &&other) = delete;

  /**
   * @brief Copy operator deleted.
   */
  GeometryStore &operator=(GeometryStore const &other) = delete;

  /**
   * @brief Move operator deleted.
   */
  GeometryStore &operator=(GeometryStore &&other) = delete;

  double_t *store_;                                      ///< Memory block containing data.
  uint32_t points3d_;                                    ///< Number of 3D points in the store.
  uint32_t capacity3d_;                                  ///< Size of the store in 3D points.
  std::map<lane::LaneId, GeometryStoreItem> lane_items_; ///< Description of lane items in the store.
};

} // namespace access
} // namespace map
} // namespace ad
