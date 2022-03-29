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

#include "ad/map/lane/GeoBorderValidInputRange.hpp"

TEST(GeoBorderValidInputRangeTests, testValidInputRange)
{
  ::ad::map::lane::GeoBorder value;
  ::ad::map::point::GeoEdge valueLeft;
  ::ad::map::point::GeoPoint valueLeftElement;
  ::ad::map::point::Longitude valueLeftElementLongitude(-180);
  valueLeftElement.longitude = valueLeftElementLongitude;
  ::ad::map::point::Latitude valueLeftElementLatitude(-90);
  valueLeftElement.latitude = valueLeftElementLatitude;
  ::ad::map::point::Altitude valueLeftElementAltitude(-11000);
  valueLeftElement.altitude = valueLeftElementAltitude;
  valueLeft.resize(1, valueLeftElement);
  value.left = valueLeft;
  ::ad::map::point::GeoEdge valueRight;
  ::ad::map::point::GeoPoint valueRightElement;
  ::ad::map::point::Longitude valueRightElementLongitude(-180);
  valueRightElement.longitude = valueRightElementLongitude;
  ::ad::map::point::Latitude valueRightElementLatitude(-90);
  valueRightElement.latitude = valueRightElementLatitude;
  ::ad::map::point::Altitude valueRightElementAltitude(-11000);
  valueRightElement.altitude = valueRightElementAltitude;
  valueRight.resize(1, valueRightElement);
  value.right = valueRight;
  ASSERT_TRUE(withinValidInputRange(value));
}
