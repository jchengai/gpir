// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/config/MapConfigFileHandler.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <fstream>

#include "ad/map/access/Logging.hpp"
#include "ad/map/point/GeoOperation.hpp"

inline std::istream &operator>>(std::istream &input,
                                ::ad::physics::Distance &distance) {
  double dummy;
  if (input >> dummy) {
    distance = ::ad::physics::Distance(dummy);
  }
  return input;
}

inline std::istream &operator>>(std::istream &input,
                                ::ad::map::point::Latitude &latitude) {
  double dummy;
  if (input >> dummy) {
    latitude = ::ad::map::point::Latitude(dummy);
  }
  return input;
}

inline std::istream &operator>>(std::istream &input,
                                ::ad::map::point::Longitude &longitude) {
  double dummy;
  if (input >> dummy) {
    longitude = ::ad::map::point::Longitude(dummy);
  }
  return input;
}

inline std::istream &operator>>(std::istream &input,
                                ::ad::map::point::Altitude &altitude) {
  double dummy;
  if (input >> dummy) {
    altitude = ::ad::map::point::Altitude(dummy);
  }
  return input;
}

namespace ad {
namespace map {
namespace config {

bool MapConfigFileHandler::readConfig(std::string const &configFileName) {
  reset();
  if (!parseConfigFile(configFileName)) {
    reset();
    return false;
  }
  return true;
}

bool MapConfigFileHandler::isInitialized() const {
  return !mConfigFileName.empty();
}

std::vector<PointOfInterest> const &MapConfigFileHandler::pointsOfInterest()
    const {
  return mPointsOfInterest;
}

bool MapConfigFileHandler::isInitializedWithFilename(
    std::string const &configFileName) const {
  auto path =
      boost::filesystem::canonical(boost::filesystem::path(configFileName));
  auto const isEqual = (path.string() == mConfigFileName);

  if (!isEqual) {
    access::getLogger()->warn(
        "AdMapAccess already initialized with a different config. Present {}, "
        "Requested {}",
        mConfigFileName, path.string());
  }

  return isEqual;
}

void MapConfigFileHandler::updateFilenameAndPath(
    std::string const &configFileName) {
  auto path =
      boost::filesystem::canonical(boost::filesystem::path(configFileName));
  mConfigFileName = path.string();
  mBaseDir = path.parent_path().string();
}

bool MapConfigFileHandler::parseConfigFile(std::string const &configFileName) {
  namespace po = boost::program_options;
  po::variables_map vm;
  po::options_description options;
  // clang-format off
  options.add_options()("ADMap.map", po::value<std::string>(), "AD map")
                       ("ADMap.openDriveOverlapMargin", po::value<std::string>(), "OpenDrive Map reader margin for overlap calculation")
                       ("ADMap.openDriveDefaultIntersectionType", po::value<std::string>(), "OpenDrive Map default intersection type")
                       ("ADMap.openDriveDefaultTrafficLightType", po::value<std::string>(), "OpenDrive Map default traffic light type (only relevant for IntersectionType::TrafficLight)")
                       ("POI.poi", po::value<std::vector<std::string>>(), "Points of interest")
                       ("ENUReference.default", po::value<std::string>(), "Default ENU reference point");
  // clang-format on

  try {
    std::ifstream settingsFile(configFileName);
    if (!settingsFile.is_open()) {
      access::getLogger()->error("Cannot open config file for reading: {}",
                                 configFileName);
      return false;
    }
    access::getLogger()->trace("Reading config file {}", configFileName);
    po::store(po::parse_config_file(settingsFile, options), vm);
    po::notify(vm);

    // we need the correct mBaseDir before we can construct any filename for the
    // maps
    updateFilenameAndPath(configFileName);

    if (vm.count("ADMap.map")) {
      MapEntry mapEntry;
      auto const mapFilename = vm["ADMap.map"].as<std::string>();
      boost::filesystem::path path(mBaseDir);
      path /= boost::filesystem::path(mapFilename);
      path = boost::filesystem::canonical(path);
      if (!boost::starts_with(path.string(), mBaseDir)) {
        access::getLogger()->error(
            "Given map file {} seems not to be located below current "
            "configuration directory {} "
            "concatenating results in: {}",
            mapFilename, mBaseDir, path.string());
        return false;
      }
      mapEntry.filename = path.string();

      physics::Distance openDriveOverlapMargin{0.};
      if (vm.count("ADMap.openDriveOverlapMargin")) {
        auto const overlapMarginString =
            vm["ADMap.openDriveOverlapMargin"].as<std::string>();
        std::istringstream input{overlapMarginString};
        if (!(input >> openDriveOverlapMargin)) {
          access::getLogger()->warn("Error extracting openDriveOverlapMargin");
          return false;
        }
      }
      mapEntry.openDriveOverlapMargin = openDriveOverlapMargin;

      intersection::IntersectionType openDriveDefaultIntersectionType =
          intersection::IntersectionType::Unknown;
      if (vm.count("ADMap.openDriveDefaultIntersectionType")) {
        auto const openDriveDefaultIntersectionTypeString =
            vm["ADMap.openDriveDefaultIntersectionType"].as<std::string>();
        openDriveDefaultIntersectionType =
            fromString<intersection::IntersectionType>(
                openDriveDefaultIntersectionTypeString);
      }
      mapEntry.openDriveDefaultIntersectionType =
          openDriveDefaultIntersectionType;

      if (mapEntry.openDriveDefaultIntersectionType ==
          intersection::IntersectionType::TrafficLight) {
        landmark::TrafficLightType openDriveDefaultTrafficLightType =
            landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN;
        if (vm.count("ADMap.openDriveDefaultTrafficLightType")) {
          auto const openDriveDefaultTrafficLightTypeString =
              vm["ADMap.openDriveDefaultTrafficLightType"].as<std::string>();
          openDriveDefaultTrafficLightType =
              fromString<landmark::TrafficLightType>(
                  openDriveDefaultTrafficLightTypeString);
        }
        mapEntry.openDriveDefaultTrafficLightType =
            openDriveDefaultTrafficLightType;
      }

      mAdMapEntry = mapEntry;
    }

    if (vm.count("POI.poi")) {
      auto const &entries = vm["POI.poi"].as<std::vector<std::string>>();
      for (auto const &entry : entries) {
        if (!parsePointOfIntereset(entry)) {
          access::getLogger()->warn(
              "Invalid  POI poi entry in config file: {}, Entry: {}",
              configFileName, entry);
          return false;
        }
      }
    }

    if (vm.count("ENUReference.default")) {
      auto const &entry = vm["ENUReference.default"].as<std::string>();
      if (!parseENUReference(entry)) {
        access::getLogger()->warn(
            "Invalid default ENU reference entry in config file: {}, Entry: {}",
            configFileName, entry);
        return false;
      }
    }
  } catch (std::exception const &e) {
    access::getLogger()->error("Error while parsing {}, Reason: {}",
                               configFileName, e.what());
    return false;
  }
  return true;
}

bool MapConfigFileHandler::parsePointOfIntereset(std::string const &entry) {
  std::istringstream input{entry};
  PointOfInterest poi;
  if (!(input >> poi.name)) {
    access::getLogger()->warn("Error extracting name of poi!");
    return false;
  }
  if (!(input >> poi.geoPoint.latitude)) {
    access::getLogger()->warn("Error extracting lat");
    return false;
  }
  if (!(input >> poi.geoPoint.longitude)) {
    access::getLogger()->warn("Error extracting lon");
    return false;
  }
  if (!(input >> poi.geoPoint.altitude)) {
    access::getLogger()->warn("Error extracting altitude");
    return false;
  }
  for (auto existingEntry : mPointsOfInterest) {
    if (existingEntry.name == poi.name) {
      access::getLogger()->warn("POI defined twice: {}", existingEntry.name);
      return false;
    }
  }
  mPointsOfInterest.push_back(poi);

  return true;
}

bool MapConfigFileHandler::parseENUReference(std::string const &entry) {
  std::istringstream input{entry};
  point::GeoPoint enuReference;
  if (!(input >> enuReference.latitude)) {
    access::getLogger()->warn("Error extracting lat");
    return false;
  }
  if (!(input >> enuReference.longitude)) {
    access::getLogger()->warn("Error extracting lon");
    return false;
  }
  if (!(input >> enuReference.altitude)) {
    access::getLogger()->warn("Error extracting altitude");
    return false;
  }
  mDefaultEnuReference = enuReference;
  return true;
}

std::string const &MapConfigFileHandler::configFileName() const {
  return mConfigFileName;
}

MapEntry const &MapConfigFileHandler::adMapEntry() const { return mAdMapEntry; }

void MapConfigFileHandler::reset() {
  mConfigFileName = "";
  mAdMapEntry = MapEntry();
  mPointsOfInterest.clear();
  mDefaultEnuReference = point::GeoPoint();
  mBaseDir.clear();
}

point::GeoPoint MapConfigFileHandler::defaultEnuReference() const {
  return mDefaultEnuReference;
}

bool MapConfigFileHandler::defaultEnuReferenceAvailable() const {
  return point::isValid(mDefaultEnuReference, false);
}

}  // namespace config
}  // namespace map
}  // namespace ad
