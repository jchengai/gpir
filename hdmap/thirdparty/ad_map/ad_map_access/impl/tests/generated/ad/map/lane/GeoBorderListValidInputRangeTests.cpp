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

#include "ad/map/lane/GeoBorderListValidInputRange.hpp"

TEST(GeoBorderListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::lane::GeoBorderList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(GeoBorderListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::lane::GeoBorderList value;
  ::ad::map::lane::GeoBorder element;
  ::ad::map::point::GeoEdge elementLeft;
  ::ad::map::point::GeoPoint elementLeftElement;
  ::ad::map::point::Longitude elementLeftElementLongitude(-180);
  elementLeftElement.longitude = elementLeftElementLongitude;
  ::ad::map::point::Latitude elementLeftElementLatitude(-90);
  elementLeftElement.latitude = elementLeftElementLatitude;
  ::ad::map::point::Altitude elementLeftElementAltitude(-11000);
  elementLeftElement.altitude = elementLeftElementAltitude;
  elementLeft.resize(1, elementLeftElement);
  element.left = elementLeft;
  ::ad::map::point::GeoEdge elementRight;
  ::ad::map::point::GeoPoint elementRightElement;
  ::ad::map::point::Longitude elementRightElementLongitude(-180);
  elementRightElement.longitude = elementRightElementLongitude;
  ::ad::map::point::Latitude elementRightElementLatitude(-90);
  elementRightElement.latitude = elementRightElementLatitude;
  ::ad::map::point::Altitude elementRightElementAltitude(-11000);
  elementRightElement.altitude = elementRightElementAltitude;
  elementRight.resize(1, elementRightElement);
  element.right = elementRight;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}
