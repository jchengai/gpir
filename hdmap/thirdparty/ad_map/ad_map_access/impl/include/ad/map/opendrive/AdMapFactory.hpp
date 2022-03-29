// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2019-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <opendrive/types.hpp>
#include <string>

#include "ad/map/access/Factory.hpp"
#include "ad/map/intersection/Types.hpp"
#include "ad/map/restriction/Types.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace opendrive */
namespace opendrive {

/**
 * @class AdMapFactory
 * @brief Creates an AdMap from an OpenDRIVE map file
 *
 * Calls the OpenDRIVE map parser and the geometry generator.
 * Once these operations are done, converts the resulting map into AdMap
 * The AdMap is generated in the store via factory calls.
 *
 */
class AdMapFactory : public access::Factory {
 public:
  explicit AdMapFactory(access::Store &store);

  AdMapFactory() = delete;
  AdMapFactory(AdMapFactory const &) = delete;
  AdMapFactory(AdMapFactory const &&) = delete;
  AdMapFactory &operator=(AdMapFactory &&) = delete;
  AdMapFactory &operator=(AdMapFactory const &) = delete;

  /**
   * @brief Reads the OpenDRIVE map xml, generates a lane map and populates the
   * AdMap store with it via admap Factory.
   *
   * @param[in] overlapMargin margin the lanes are narrowed when calculating
   * overlaps.
   * @param[in] defaultIntersectionType default type of intersections to be used
   * @param[in] defaultTrafficLightType default type of traffic light to be used
   * in case of traffic light intersection
   *
   * @returns \c true if a map was generated, false otherwise.
   */
  bool createAdMap(std::string const &mapFilePath, double const overlapMargin,
                   intersection::IntersectionType const defaultIntersectionType,
                   landmark::TrafficLightType const defaultTrafficLightType =
                       landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN);

  /**
   * @brief Parses OpenDRIVE map content string, generates a lane map and
   * populates the AdMap store with it via admap Factory.
   *
   * @param[in] overlapMargin margin the lanes are narrowed when calculating
   * overlaps.
   * @param[in] defaultIntersectionType default type of intersections to be used
   * @param[in] defaultTrafficLightType default type of traffic light to be used
   * in case of traffic light intersection
   *
   * @returns \c true if a map was generated, false otherwise.
   */
  bool createAdMapFromString(
      std::string const &mapContent, double const overlapMargin,
      intersection::IntersectionType const defaultIntersectionType,
      landmark::TrafficLightType const defaultTrafficLightType =
          landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN);

  /**
   * @brief Returns true if the given file matches an OpenDRIVE map file.
   * @param[in] mapName The path to the file.
   * @returns \c true if the map is generated without any fatal error
   */
  static bool isOpenDriveMap(std::string const &mapName);

 private:
  /**
   * @brief Generates a lane map and populates the AdMap store with previously
   * parsed OpenDRIVE data.
   *
   * @param[in] overlapMargin margin the lanes are narrowed when calculating
   * overlaps.
   * @param[in] defaultIntersectionType default type of intersections to be used
   * @param[in] defaultTrafficLightType default type of traffic light to be used
   * in case of traffic light intersection
   *
   * @returns \c true if a map was generated, false otherwise.
   */
  bool createAdMap(::opendrive::OpenDriveData &openDriveData,
                   double const overlapMargin,
                   intersection::IntersectionType const defaultIntersectionType,
                   landmark::TrafficLightType const defaultTrafficLightType);

  /**
   * @brief Generates an AdMap using the extended map data generated from the
   * openDrive data.
   * @param[in,out] mapData The OpenDRIVE map data container
   * @param[in] defaultIntersectionType default type of intersections to be used
   * @param[in] defaultTrafficLightType default type of traffic light to be used
   * in case of traffic light intersection
   *
   * @returns \c true if the map is generated in the AdMap store without any
   * fatal error
   */
  bool convertToAdMap(
      ::opendrive::OpenDriveData &mapData,
      intersection::IntersectionType const defaultIntersectionType,
      landmark::TrafficLightType const defaultTrafficLightType);

  /**
   * @brief Creates default access restrictions, i.e. all vehicles are allowed.
   */
  restriction::Restrictions createRoadRestrictions() const;

  /**
   * @brief Reads the parametric speed from the given lane and updates the store
   * accordlingy
   * @param[in] lane The lane from which the lane speed will be readed
   * @returns \c true if the information is successfully updated in the store
   */
  bool setLaneSpeed(::opendrive::Lane const &lane);

  /**
   * @brief Adds a landmark to the admap store
   * @param[in] landmark The landmark from which the information will be readed
   * @returns \c true if successfully added to the store
   */
  bool addLandmark(::opendrive::Landmark const &landmark);

  /**
   * @brief Adds the lane to the admap store
   * @param[in] lane ::opendrive::Lane data
   * @returns \c true if successfully added to the store
   */
  bool addLane(::opendrive::Lane const &lane);

  /**
   * @brief Updates the admap store with the contacts for the given lane
   * @param[in] lane lane to be modified
   * @param[in] landmark landmark info in case the contact requires it
   * @param[in] location specifies the location where the contact takes place
   * @returns \c true if successfully added to the store
   */
  bool addSpecialContact(::opendrive::Lane const &lane,
                         ::opendrive::Landmark const &landmark,
                         lane::ContactLocation const &location);

  /**
   * @brief Return true if a vehicle can drive on this lane
   * @param[in] laneType
   * @returns \c true if it is possible to drive on this lane type
   */
  bool isDrivableLane(lane::LaneType laneType) const;

  /**
   * @brief Iterates over the traffic signals and traffic signs and adds the
   * info the store
   * @param[in] lane The lane holds a list of all the landmarks that will be
   * added
   * @param[in] landmarks A map of all the landmarks present in the current map
   * @returns \c true if successfully added to the store
   */
  bool addSpecialContacts(::opendrive::Lane const &lane,
                          ::opendrive::LandmarkMap const &landmarks);

  /**
   * @brief Adds the successor, predecessor and neighbor contacts to the admap
   * store for the given lane
   * @param[in] lane The lane to be modified
   * @param[in] defaultIntersectionType default type of intersections to be used
   * @param[in] defaultTrafficLightType default type of traffic light to be used
   * in case of traffic light intersection
   *
   * @returns \c true if successfully added to the store
   */
  bool addContactLanes(
      ::opendrive::Lane const &lane,
      intersection::IntersectionType const defaultIntersectionType,
      landmark::TrafficLightType const defaultTrafficLightType);
};

}  // namespace opendrive
}  // namespace map
}  // namespace ad
