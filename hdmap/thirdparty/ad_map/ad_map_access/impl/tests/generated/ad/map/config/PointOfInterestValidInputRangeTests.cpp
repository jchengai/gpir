/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2018-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

/*
 * Generated file
 */

#include <gtest/gtest.h>

#include <limits>

#include "ad/map/config/PointOfInterestValidInputRange.hpp"

TEST(PointOfInterestValidInputRangeTests, testValidInputRange)
{
  ::ad::map::config::PointOfInterest value;
  ::ad::map::point::GeoPoint valueGeoPoint;
  ::ad::map::point::Longitude valueGeoPointLongitude(-180);
  valueGeoPoint.longitude = valueGeoPointLongitude;
  ::ad::map::point::Latitude valueGeoPointLatitude(-90);
  valueGeoPoint.latitude = valueGeoPointLatitude;
  ::ad::map::point::Altitude valueGeoPointAltitude(-11000);
  valueGeoPoint.altitude = valueGeoPointAltitude;
  value.geoPoint = valueGeoPoint;
  std::string valueName{"min"};
  value.name = valueName;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(PointOfInterestValidInputRangeTests, testValidInputRangeGeoPointTooSmall)
{
  ::ad::map::config::PointOfInterest value;
  ::ad::map::point::GeoPoint valueGeoPoint;
  ::ad::map::point::Longitude valueGeoPointLongitude(-180);
  valueGeoPoint.longitude = valueGeoPointLongitude;
  ::ad::map::point::Latitude valueGeoPointLatitude(-90);
  valueGeoPoint.latitude = valueGeoPointLatitude;
  ::ad::map::point::Altitude valueGeoPointAltitude(-11000);
  valueGeoPoint.altitude = valueGeoPointAltitude;
  value.geoPoint = valueGeoPoint;
  std::string valueName{"min"};
  value.name = valueName;

  // override member with data type value below input range minimum
  ::ad::map::point::GeoPoint invalidInitializedMember;
  ::ad::map::point::Longitude invalidInitializedMemberLongitude(-180 * 1.1);
  invalidInitializedMember.longitude = invalidInitializedMemberLongitude;
  value.geoPoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(PointOfInterestValidInputRangeTests, testValidInputRangeGeoPointTooBig)
{
  ::ad::map::config::PointOfInterest value;
  ::ad::map::point::GeoPoint valueGeoPoint;
  ::ad::map::point::Longitude valueGeoPointLongitude(-180);
  valueGeoPoint.longitude = valueGeoPointLongitude;
  ::ad::map::point::Latitude valueGeoPointLatitude(-90);
  valueGeoPoint.latitude = valueGeoPointLatitude;
  ::ad::map::point::Altitude valueGeoPointAltitude(-11000);
  valueGeoPoint.altitude = valueGeoPointAltitude;
  value.geoPoint = valueGeoPoint;
  std::string valueName{"min"};
  value.name = valueName;

  // override member with data type value above input range maximum
  ::ad::map::point::GeoPoint invalidInitializedMember;
  ::ad::map::point::Longitude invalidInitializedMemberLongitude(180 * 1.1);
  invalidInitializedMember.longitude = invalidInitializedMemberLongitude;
  value.geoPoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
