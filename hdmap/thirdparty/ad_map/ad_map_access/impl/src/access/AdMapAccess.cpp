// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "AdMapAccess.hpp"

#include <spdlog/sinks/stdout_color_sinks.h>

#include "ad/map/opendrive/AdMapFactory.hpp"
#include "ad/map/point/Operation.hpp"
#include "ad/map/serialize/SerializerFileCRC32.hpp"

namespace ad {
namespace map {
namespace access {

typedef std::lock_guard<std::recursive_mutex> LockGuard;

AdMapAccess &AdMapAccess::getAdMapAccessInstance() {
  static AdMapAccess singleton;
  return singleton;
}

AdMapAccess &AdMapAccess::getInitializedInstance() {
  AdMapAccess &singleton = getAdMapAccessInstance();
  LockGuard guard(singleton.mMutex);
  if (!singleton.mConfigFileHandler.isInitialized() &&
      !singleton.mInitializedFromStore) {
    throw std::runtime_error(
        "Instance not initialized. Forgot to call AdMapAccess::initialize?");
  }
  return singleton;
}

AdMapAccess::AdMapAccess() {
  reset();

  mLogger = spdlog::stdout_color_mt("ad_map_access");
  mLogger->set_level(spdlog::level::warn);
}

AdMapAccess::~AdMapAccess() {}

bool AdMapAccess::initialize(std::string const &configFileName) {
  LockGuard guard(mMutex);
  if (mInitializedFromStore) {
    mLogger->error(
        "AdMapAccess::initialize(config) failed; already initialized from "
        "store");
    return false;
  }
  if (mConfigFileHandler.isInitialized()) {
    if (!mConfigFileHandler.isInitializedWithFilename(configFileName)) {
      mLogger->error(
          "AdMapAccess::initialize(config) with file {} failed; already "
          "initialized with different config: {}",
          configFileName, mConfigFileHandler.configFileName());
      return false;
    }
    return true;
  }
  if (!mConfigFileHandler.readConfig(configFileName)) {
    mLogger->warn("AdMapAccess::initialize(config) Failed to open {}",
                  configFileName);
    return false;
  }

  mLogger->info("AdMapAccess::initialize(config) Successfully opened {}",
                configFileName);
  if (!readMap(mConfigFileHandler.adMapEntry().filename)) {
    mLogger->warn("Unable to read map {}",
                  mConfigFileHandler.adMapEntry().filename);
    reset();
    return false;
  }
  if (mConfigFileHandler.defaultEnuReferenceAvailable()) {
    setENUReferencePoint(mConfigFileHandler.defaultEnuReference());
  }
  return true;
}

bool AdMapAccess::initializeFromOpenDriveContent(
    std::string const &openDriveContent, double const overlapMargin,
    intersection::IntersectionType const defaultIntersectionType,
    landmark::TrafficLightType const defaultTrafficLightType) {
  LockGuard guard(mMutex);
  if (mConfigFileHandler.isInitialized()) {
    mLogger->error(
        "AdMapAccess::initializeFromOpenDriveContent() failed; already "
        "initialized with different config: {}",
        mConfigFileHandler.configFileName());
    return false;
  }

  auto const openDriveContentChecksum = serialize::calculateCRC32Checksum(
      openDriveContent.c_str(), openDriveContent.size());
  if (mInitializedFromStore) {
    if (openDriveContentChecksum == mInitializeFromOpenDriveContentChecksum) {
      // initialize twice from same store object is ok.
      return true;
    } else {
      mLogger->error(
          "AdMapAccess::initializeFromOpenDriveContent() failed; already "
          "initialized with different store "
          "object or content");
      return false;
    }
  }

  auto store = std::make_shared<Store>();
  opendrive::AdMapFactory factory(*store);

  bool result = factory.createAdMapFromString(openDriveContent, overlapMargin,
                                              defaultIntersectionType,
                                              defaultTrafficLightType);
  if (result) {
    mInitializeFromOpenDriveContentChecksum = openDriveContentChecksum;
    mInitializedFromStore = true;
    mStore = store;
  } else {
    mLogger->error(
        "AdMapAccess::initializeFromOpenDriveContent() failed; error parsing "
        "OpenDRIVE content");
  }
  return result;
}

bool AdMapAccess::initialize(Store::Ptr store) {
  LockGuard guard(mMutex);
  if (!bool(store) || !store->isValid()) {
    mLogger->error("AdMapAccess::initialize(store) provided store is invalid");
    return false;
  }
  if (mConfigFileHandler.isInitialized()) {
    mLogger->error(
        "AdMapAccess::initialize(store) failed; already initialized with "
        "different config {}",
        mConfigFileHandler.configFileName());
    return false;
  }

  if (mInitializedFromStore) {
    if (mStore != store) {
      mLogger->error(
          "AdMapAccess::initialize(store) failed; already initialized with "
          "different store object");
      return false;
    } else {
      // initialize twice from same store object is ok.
      return true;
    }
  }
  mInitializedFromStore = true;

  mStore = store;
  auto boundingSphere = mStore->getBoundingSphere();
  auto centerGeo = point::toGeo(boundingSphere.center);
  setENUReferencePoint(centerGeo);

  mLogger->info("AdMapAccess::initialized from store");
  return true;
}

void AdMapAccess::reset() {
  LockGuard guard(mMutex);
  mStore = std::make_shared<Store>();
  mConfigFileHandler.reset();
  mInitializedFromStore = false;
  mCoordinateTransform = std::make_shared<point::CoordinateTransform>();
}

bool AdMapAccess::readMap(std::string const &mapName) {
  if (ad::map::opendrive::AdMapFactory::isOpenDriveMap(mapName)) {
    return readOpenDriveMap(mapName);
  } else {
    return readAdMap(mapName);
  }
}

bool AdMapAccess::readOpenDriveMap(std::string const &mapName) {
  opendrive::AdMapFactory factory(*mStore);
  return factory.createAdMap(
      mapName,
      static_cast<double>(
          mConfigFileHandler.adMapEntry().openDriveOverlapMargin),
      mConfigFileHandler.adMapEntry().openDriveDefaultIntersectionType,
      mConfigFileHandler.adMapEntry().openDriveDefaultTrafficLightType);
}

bool AdMapAccess::readAdMap(std::string const &mapName) {
  serialize::SerializerFileCRC32 serializer(false);
  size_t version_major = 0;
  size_t version_minor = 0;
  if (!serializer.open(mapName.c_str(), version_major, version_minor)) {
    mLogger->warn("Unable to open map for reading {}", mapName);
    return false;
  }
  mLogger->info("Opened map {}", mapName);
  mLogger->trace("Version {}.{}", version_major, version_minor);
  if (!mStore->load(serializer)) {
    mLogger->warn("Unable to read map {}", mapName);
    return false;
  }
  if (!serializer.close()) {
    mLogger->warn("Map file is corrupt {}", mapName);
    return false;
  }

  return true;
}

}  // namespace access
}  // namespace map
}  // namespace ad
