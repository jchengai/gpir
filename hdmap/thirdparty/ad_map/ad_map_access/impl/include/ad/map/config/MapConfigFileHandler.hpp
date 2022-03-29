// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <vector>
#include "ad/map/config/MapEntry.hpp"
#include "ad/map/config/PointOfInterest.hpp"
#include "ad/map/point/GeoPoint.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace config */
namespace config {

/**
 * @class MapConfigFileHandler
 * @brief Parse config file that specifies all known maps
 *
 * The config file specifies which maps to use by the AdMapAccess class.
 * It has three sections:
 * - ADMap
 * - POI
 * - ENUReference
 *
 * ADMap specifies a map and optional parameters for loading. An entry is given with
 * - map (mandatory): filename of the adm or OpenDrive map file, relative path below in the directory of the config file
 * itself. The filename must not have blanks.
 * - openDriveOverlapMargin (optional): OpenDrive Map reader margin for overlap calculation
 * - openDriveDefaultIntersectionType (optional): OpenDrive Map default intersection type
 *
 * The positions define the covered area of the file. Example:
 * [ADMap]
 * map=filename
 *
 * POI specifies a [list of] point[s] of interest. An entry can provide one or more POI descriptions.
 * Whereas a POI is given with:
 * - a name of the poi
 * - latitude, longitude and altitude values of the POI
 *
 * Example:
 * [POI]
 * poi=One 49.0189305 8.4399515 0.
 * poi=Two 49.0191653 8.4401407 0.
 * poi=Three 49.0192038 8.4401582 0.
 * poi=Four 49.0192092 8.4401439 0.
 *
 * All values are given in decimal degrees, e.g. 48.405
 *
 * In addition there is the possibility to provide a default ENUReference point to be set automatically when loading the
 * map file.
 * The default ENU Reference point is given by:
 * - latitude, longitude and altitude values of the reference point
 *
 * Example
 * [ENUReference]
 * default=49.0192671 8.4421163 0
 *
 * When parsing the config file, the existance/correctness of the map file itself will not be checked.
 */
class MapConfigFileHandler
{
public:
  MapConfigFileHandler() = default;

  // Copy operators and constructors are deleted to avoid accidential copies
  MapConfigFileHandler(MapConfigFileHandler const &) = delete;
  MapConfigFileHandler(MapConfigFileHandler &&) = delete;
  MapConfigFileHandler &operator=(MapConfigFileHandler &&) = delete;
  MapConfigFileHandler &operator=(MapConfigFileHandler const &) = delete;

  /**
   * @brief read configuration from given configFileName
   *
   * @return true on successful completion
   * @return false if configFileName cannot be opened or has invalid syntax
   */
  bool readConfig(std::string const &configFileName);

  //! @return true if a valid configuration was read
  bool isInitialized() const;

  //! @return true if configuration was done with given file name
  bool isInitializedWithFilename(std::string const &configFileName) const;

  //! @return name of loaded config file
  std::string const &configFileName() const;

  //! @return configured MapEntry
  MapEntry const &adMapEntry() const;

  //! @return list of POIs
  std::vector<PointOfInterest> const &pointsOfInterest() const;

  //! @return the default Enu reference point
  point::GeoPoint defaultEnuReference() const;

  //! @return \c 'true' if the default Enu reference point is existing
  bool defaultEnuReferenceAvailable() const;

  void reset();

private:
  std::string mConfigFileName{}; //!< name of the loaded config file
  std::string mBaseDir{};        //!< name of directory where config file is located

  MapEntry mAdMapEntry;
  std::vector<PointOfInterest> mPointsOfInterest;
  point::GeoPoint mDefaultEnuReference;

  void updateFilenameAndPath(std::string const &configFileName);
  bool parseConfigFile(std::string const &configFileName);
  bool parsePointOfIntereset(std::string const &entry);
  bool parseENUReference(std::string const &entry);
};

} // namespace config
} // namespace map
} // namespace ad
