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

#include "ad/map/match/ENUObjectPositionListValidInputRange.hpp"

TEST(ENUObjectPositionListValidInputRangeTests, testValidInputRangeLowerThanInputRangeMin)
{
  ::ad::map::match::ENUObjectPositionList value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUObjectPositionListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::match::ENUObjectPositionList value;
  ::ad::map::match::ENUObjectPosition element;
  ::ad::map::point::ENUPoint elementCenterPoint;
  ::ad::map::point::ENUCoordinate elementCenterPointX(-16384);
  elementCenterPoint.x = elementCenterPointX;
  ::ad::map::point::ENUCoordinate elementCenterPointY(-16384);
  elementCenterPoint.y = elementCenterPointY;
  ::ad::map::point::ENUCoordinate elementCenterPointZ(-16384);
  elementCenterPoint.z = elementCenterPointZ;
  element.centerPoint = elementCenterPoint;
  ::ad::map::point::ENUHeading elementHeading(-3.141592655);
  element.heading = elementHeading;
  ::ad::map::point::GeoPoint elementEnuReferencePoint;
  ::ad::map::point::Longitude elementEnuReferencePointLongitude(-180);
  elementEnuReferencePoint.longitude = elementEnuReferencePointLongitude;
  ::ad::map::point::Latitude elementEnuReferencePointLatitude(-90);
  elementEnuReferencePoint.latitude = elementEnuReferencePointLatitude;
  ::ad::map::point::Altitude elementEnuReferencePointAltitude(-11000);
  elementEnuReferencePoint.altitude = elementEnuReferencePointAltitude;
  element.enuReferencePoint = elementEnuReferencePoint;
  ::ad::physics::Dimension3D elementDimension;
  ::ad::physics::Distance elementDimensionLength(-1e9);
  elementDimension.length = elementDimensionLength;
  ::ad::physics::Distance elementDimensionWidth(-1e9);
  elementDimension.width = elementDimensionWidth;
  ::ad::physics::Distance elementDimensionHeight(-1e9);
  elementDimension.height = elementDimensionHeight;
  element.dimension = elementDimension;
  value.resize(1, element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUObjectPositionListValidInputRangeTests, testValidInputRangeElementTypeInvalid)
{
  ::ad::map::match::ENUObjectPositionList value;
  ::ad::map::match::ENUObjectPosition element;
  ::ad::map::point::ENUPoint elementCenterPoint;
  ::ad::map::point::ENUCoordinate elementCenterPointX(-16384 * 1.1);
  elementCenterPoint.x = elementCenterPointX;
  element.centerPoint = elementCenterPoint;
  value.resize(1, element);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUObjectPositionListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::match::ENUObjectPositionList value;
  ::ad::map::match::ENUObjectPosition element;
  ::ad::map::point::ENUPoint elementCenterPoint;
  ::ad::map::point::ENUCoordinate elementCenterPointX(-16384);
  elementCenterPoint.x = elementCenterPointX;
  ::ad::map::point::ENUCoordinate elementCenterPointY(-16384);
  elementCenterPoint.y = elementCenterPointY;
  ::ad::map::point::ENUCoordinate elementCenterPointZ(-16384);
  elementCenterPoint.z = elementCenterPointZ;
  element.centerPoint = elementCenterPoint;
  ::ad::map::point::ENUHeading elementHeading(-3.141592655);
  element.heading = elementHeading;
  ::ad::map::point::GeoPoint elementEnuReferencePoint;
  ::ad::map::point::Longitude elementEnuReferencePointLongitude(-180);
  elementEnuReferencePoint.longitude = elementEnuReferencePointLongitude;
  ::ad::map::point::Latitude elementEnuReferencePointLatitude(-90);
  elementEnuReferencePoint.latitude = elementEnuReferencePointLatitude;
  ::ad::map::point::Altitude elementEnuReferencePointAltitude(-11000);
  elementEnuReferencePoint.altitude = elementEnuReferencePointAltitude;
  element.enuReferencePoint = elementEnuReferencePoint;
  ::ad::physics::Dimension3D elementDimension;
  ::ad::physics::Distance elementDimensionLength(-1e9);
  elementDimension.length = elementDimensionLength;
  ::ad::physics::Distance elementDimensionWidth(-1e9);
  elementDimension.width = elementDimensionWidth;
  ::ad::physics::Distance elementDimensionHeight(-1e9);
  elementDimension.height = elementDimensionHeight;
  element.dimension = elementDimension;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUObjectPositionListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::match::ENUObjectPositionList value;
  ::ad::map::match::ENUObjectPosition element;
  ::ad::map::point::ENUPoint elementCenterPoint;
  ::ad::map::point::ENUCoordinate elementCenterPointX(-16384 * 1.1);
  elementCenterPoint.x = elementCenterPointX;
  element.centerPoint = elementCenterPoint;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
