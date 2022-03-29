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

#include "ad/map/point/GeoPointValidInputRange.hpp"

TEST(GeoPointValidInputRangeTests, testValidInputRange)
{
  ::ad::map::point::GeoPoint value;
  ::ad::map::point::Longitude valueLongitude(-180);
  value.longitude = valueLongitude;
  ::ad::map::point::Latitude valueLatitude(-90);
  value.latitude = valueLatitude;
  ::ad::map::point::Altitude valueAltitude(-11000);
  value.altitude = valueAltitude;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(GeoPointValidInputRangeTests, testValidInputRangeLongitudeTooSmall)
{
  ::ad::map::point::GeoPoint value;
  ::ad::map::point::Longitude valueLongitude(-180);
  value.longitude = valueLongitude;
  ::ad::map::point::Latitude valueLatitude(-90);
  value.latitude = valueLatitude;
  ::ad::map::point::Altitude valueAltitude(-11000);
  value.altitude = valueAltitude;

  // override member with data type value below input range minimum
  ::ad::map::point::Longitude invalidInitializedMember(-180 * 1.1);
  value.longitude = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(GeoPointValidInputRangeTests, testValidInputRangeLongitudeTooBig)
{
  ::ad::map::point::GeoPoint value;
  ::ad::map::point::Longitude valueLongitude(-180);
  value.longitude = valueLongitude;
  ::ad::map::point::Latitude valueLatitude(-90);
  value.latitude = valueLatitude;
  ::ad::map::point::Altitude valueAltitude(-11000);
  value.altitude = valueAltitude;

  // override member with data type value above input range maximum
  ::ad::map::point::Longitude invalidInitializedMember(180 * 1.1);
  value.longitude = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(GeoPointValidInputRangeTests, testValidInputRangelongitudeDefault)
{
  ::ad::map::point::GeoPoint value;
  ::ad::map::point::Longitude valueDefault;
  value.longitude = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(GeoPointValidInputRangeTests, testValidInputRangeLatitudeTooSmall)
{
  ::ad::map::point::GeoPoint value;
  ::ad::map::point::Longitude valueLongitude(-180);
  value.longitude = valueLongitude;
  ::ad::map::point::Latitude valueLatitude(-90);
  value.latitude = valueLatitude;
  ::ad::map::point::Altitude valueAltitude(-11000);
  value.altitude = valueAltitude;

  // override member with data type value below input range minimum
  ::ad::map::point::Latitude invalidInitializedMember(-90 * 1.1);
  value.latitude = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(GeoPointValidInputRangeTests, testValidInputRangeLatitudeTooBig)
{
  ::ad::map::point::GeoPoint value;
  ::ad::map::point::Longitude valueLongitude(-180);
  value.longitude = valueLongitude;
  ::ad::map::point::Latitude valueLatitude(-90);
  value.latitude = valueLatitude;
  ::ad::map::point::Altitude valueAltitude(-11000);
  value.altitude = valueAltitude;

  // override member with data type value above input range maximum
  ::ad::map::point::Latitude invalidInitializedMember(90 * 1.1);
  value.latitude = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(GeoPointValidInputRangeTests, testValidInputRangelatitudeDefault)
{
  ::ad::map::point::GeoPoint value;
  ::ad::map::point::Latitude valueDefault;
  value.latitude = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(GeoPointValidInputRangeTests, testValidInputRangeAltitudeTooSmall)
{
  ::ad::map::point::GeoPoint value;
  ::ad::map::point::Longitude valueLongitude(-180);
  value.longitude = valueLongitude;
  ::ad::map::point::Latitude valueLatitude(-90);
  value.latitude = valueLatitude;
  ::ad::map::point::Altitude valueAltitude(-11000);
  value.altitude = valueAltitude;

  // override member with data type value below input range minimum
  ::ad::map::point::Altitude invalidInitializedMember(-11000 * 1.1);
  value.altitude = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(GeoPointValidInputRangeTests, testValidInputRangeAltitudeTooBig)
{
  ::ad::map::point::GeoPoint value;
  ::ad::map::point::Longitude valueLongitude(-180);
  value.longitude = valueLongitude;
  ::ad::map::point::Latitude valueLatitude(-90);
  value.latitude = valueLatitude;
  ::ad::map::point::Altitude valueAltitude(-11000);
  value.altitude = valueAltitude;

  // override member with data type value above input range maximum
  ::ad::map::point::Altitude invalidInitializedMember(9000 * 1.1);
  value.altitude = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(GeoPointValidInputRangeTests, testValidInputRangealtitudeDefault)
{
  ::ad::map::point::GeoPoint value;
  ::ad::map::point::Altitude valueDefault;
  value.altitude = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
