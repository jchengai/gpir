// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <mutex>
#include <spdlog/spdlog.h>
#include <string>
#include "ad/map/access/Store.hpp"
#include "ad/map/config/MapConfigFileHandler.hpp"
#include "ad/map/point/CoordinateTransform.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace access */
namespace access {

/**
 * @class AdMapAccess
 * @brief handle loading and removal of map data to/from a map store
 *
 * The type of map store depends on the implementation that is linked to the application.
 * There exists two variants of AdMapAcces, one for AD Map Store and one for Lanelet.
 *
 * The variants (adg::ad::map::Store, Lanelet) themselves are a singleton and therefore any consumer/user of the
 * Store can use it directly.
 */
class AdMapAccess
{
public:
  /**
   * @brief get reference of the one AdMapAccess singleton
   *
   * Before accessing the instance you have to call AdMapAccess::initialize() to initialize the static instance.
   * If the instance is not initialized when this function is called, an exception is thrown.
   *
   * @return Reference to AdMapAccess singleton
   */
  static AdMapAccess &getInitializedInstance();

  /**
   * @brief initialize singleton with given configuration file
   *
   * The configuration file specifies all available maps, see ConfigFileHandler for details of the semantics
   * @return true if initialization was successful (i.e. config file exists or singleton was already initialized)
   * @return false if config file doesn't exist or the singleton was already initialized with a different config file
   */
  bool initialize(std::string const &configFileName);

  /**
   * @brief initialize singleton with OpenDRIVE content string
   *
   * If previously initialized with the same opendrive content the function performs nothing, but returns true.
   *
   * The string contains the full OpenDRIVE data
   * @return true if initialization was successful.
   * @return false if OpenDRIVE data wasn't valid or the singleton was already initialized differently
   */
  bool initializeFromOpenDriveContent(std::string const &openDriveContent,
                                      double const overlapMargin,
                                      intersection::IntersectionType const defaultIntersectionType,
                                      landmark::TrafficLightType const defaultTrafficLightType
                                      = landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN);
  /**
   * @brief initialize singleton with given store
   *
   * No map files will be read. The store has to be filled by other means.
   */
  bool initialize(Store::Ptr store);

  /**
   * @brief remove all loaded maps and configuration
   */
  void reset();

  /**
   * @brief a coordinate transform object
   *
   * The ENURefPoint of this is set at the beginning of getMapMatchedPositions() call.
   */
  std::shared_ptr<point::CoordinateTransform> mCoordinateTransform;

  //! helper for parsing the config file
  config::MapConfigFileHandler mConfigFileHandler{};

  bool mInitializedFromStore{false};

  uint32_t mInitializeFromOpenDriveContentChecksum{0u};

  std::shared_ptr<spdlog::logger> mLogger;

  //! protect access to mLoadedMaps and used while loading a map
  std::recursive_mutex mMutex{};

  //! the store object
  Store::Ptr mStore;

  //! actually read the given map
  bool readMap(std::string const &mapName);

  bool readAdMap(std::string const &mapName);

  bool readOpenDriveMap(std::string const &mapName);

  explicit AdMapAccess();
  ~AdMapAccess();

  static AdMapAccess &getAdMapAccessInstance();

private:
  // Copy operators and constructors are deleted to avoid accidental copies
  AdMapAccess(AdMapAccess const &) = delete;
  AdMapAccess(AdMapAccess const &&) = delete;
  AdMapAccess &operator=(AdMapAccess &&) = delete;
  AdMapAccess &operator=(AdMapAccess const &) = delete;
};

} // namespace access
} // namespace map
} // namespace ad
