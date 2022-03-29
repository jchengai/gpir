// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/test_support/IntersectionTestBase.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace test_support */
namespace test_support {

/**
 * @brief Class that implements Intersection Test Base functionality for the artificial maps
 *
 * The maps only differ in the type of traffic regulation. Therefore, all locations
 * that have to be provided for the base class are the same.
 * This class also handles both right and left handed traffic versions of the map.
 */
struct ArtificialIntersectionTestBase : test_support::IntersectionTestBase
{
  ArtificialIntersectionTestBase();

  virtual ~ArtificialIntersectionTestBase() = default;

  /**
   * @brief Implements corresponding functionality for the artificial map
   */
  virtual point::GeoPoint getGeoFromNorth() const override;

  /**
   * @brief Implements corresponding functionality for the artificial map
   */
  virtual point::GeoPoint getGeoToNorth() const override;

  /**
   * @brief Implements corresponding functionality for the artificial map
   */
  virtual point::GeoPoint getGeoFromSouth() const override;

  /**
   * @brief Implements corresponding functionality for the artificial map
   */
  virtual point::GeoPoint getGeoToSouth() const override;

  /**
   * @brief Implements corresponding functionality for the artificial map
   */
  virtual point::GeoPoint getGeoFromWest() const override;

  /**
   * @brief Implements corresponding functionality for the artificial map
   */
  virtual point::GeoPoint getGeoToWest() const override;

  /**
   * @brief Implements corresponding functionality for the artificial map
   */
  virtual point::GeoPoint getGeoFromEast() const override;

  /**
   * @brief Implements corresponding functionality for the artificial map
   */
  virtual point::GeoPoint getGeoToEast() const override;

  // the points in the map of the intersection arms ordered clockwise...
  point::GeoPoint mPoint_01h;
  point::GeoPoint mPoint_02h;
  point::GeoPoint mPoint_04h;
  point::GeoPoint mPoint_05h;
  point::GeoPoint mPoint_07h;
  point::GeoPoint mPoint_08h;
  point::GeoPoint mPoint_10h;
  point::GeoPoint mPoint_11h;
};

} // namespace test_support
} // namespace map
} // namespace ad
