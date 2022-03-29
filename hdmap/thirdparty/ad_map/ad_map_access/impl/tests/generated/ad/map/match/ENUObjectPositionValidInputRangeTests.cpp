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

#include "ad/map/match/ENUObjectPositionValidInputRange.hpp"

TEST(ENUObjectPositionValidInputRangeTests, testValidInputRange)
{
  ::ad::map::match::ENUObjectPosition value;
  ::ad::map::point::ENUPoint valueCenterPoint;
  ::ad::map::point::ENUCoordinate valueCenterPointX(-16384);
  valueCenterPoint.x = valueCenterPointX;
  ::ad::map::point::ENUCoordinate valueCenterPointY(-16384);
  valueCenterPoint.y = valueCenterPointY;
  ::ad::map::point::ENUCoordinate valueCenterPointZ(-16384);
  valueCenterPoint.z = valueCenterPointZ;
  value.centerPoint = valueCenterPoint;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::point::GeoPoint valueEnuReferencePoint;
  ::ad::map::point::Longitude valueEnuReferencePointLongitude(-180);
  valueEnuReferencePoint.longitude = valueEnuReferencePointLongitude;
  ::ad::map::point::Latitude valueEnuReferencePointLatitude(-90);
  valueEnuReferencePoint.latitude = valueEnuReferencePointLatitude;
  ::ad::map::point::Altitude valueEnuReferencePointAltitude(-11000);
  valueEnuReferencePoint.altitude = valueEnuReferencePointAltitude;
  value.enuReferencePoint = valueEnuReferencePoint;
  ::ad::physics::Dimension3D valueDimension;
  ::ad::physics::Distance valueDimensionLength(-1e9);
  valueDimension.length = valueDimensionLength;
  ::ad::physics::Distance valueDimensionWidth(-1e9);
  valueDimension.width = valueDimensionWidth;
  ::ad::physics::Distance valueDimensionHeight(-1e9);
  valueDimension.height = valueDimensionHeight;
  value.dimension = valueDimension;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUObjectPositionValidInputRangeTests, testValidInputRangeCenterPointTooSmall)
{
  ::ad::map::match::ENUObjectPosition value;
  ::ad::map::point::ENUPoint valueCenterPoint;
  ::ad::map::point::ENUCoordinate valueCenterPointX(-16384);
  valueCenterPoint.x = valueCenterPointX;
  ::ad::map::point::ENUCoordinate valueCenterPointY(-16384);
  valueCenterPoint.y = valueCenterPointY;
  ::ad::map::point::ENUCoordinate valueCenterPointZ(-16384);
  valueCenterPoint.z = valueCenterPointZ;
  value.centerPoint = valueCenterPoint;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::point::GeoPoint valueEnuReferencePoint;
  ::ad::map::point::Longitude valueEnuReferencePointLongitude(-180);
  valueEnuReferencePoint.longitude = valueEnuReferencePointLongitude;
  ::ad::map::point::Latitude valueEnuReferencePointLatitude(-90);
  valueEnuReferencePoint.latitude = valueEnuReferencePointLatitude;
  ::ad::map::point::Altitude valueEnuReferencePointAltitude(-11000);
  valueEnuReferencePoint.altitude = valueEnuReferencePointAltitude;
  value.enuReferencePoint = valueEnuReferencePoint;
  ::ad::physics::Dimension3D valueDimension;
  ::ad::physics::Distance valueDimensionLength(-1e9);
  valueDimension.length = valueDimensionLength;
  ::ad::physics::Distance valueDimensionWidth(-1e9);
  valueDimension.width = valueDimensionWidth;
  ::ad::physics::Distance valueDimensionHeight(-1e9);
  valueDimension.height = valueDimensionHeight;
  value.dimension = valueDimension;

  // override member with data type value below input range minimum
  ::ad::map::point::ENUPoint invalidInitializedMember;
  ::ad::map::point::ENUCoordinate invalidInitializedMemberX(-16384 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.centerPoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUObjectPositionValidInputRangeTests, testValidInputRangeCenterPointTooBig)
{
  ::ad::map::match::ENUObjectPosition value;
  ::ad::map::point::ENUPoint valueCenterPoint;
  ::ad::map::point::ENUCoordinate valueCenterPointX(-16384);
  valueCenterPoint.x = valueCenterPointX;
  ::ad::map::point::ENUCoordinate valueCenterPointY(-16384);
  valueCenterPoint.y = valueCenterPointY;
  ::ad::map::point::ENUCoordinate valueCenterPointZ(-16384);
  valueCenterPoint.z = valueCenterPointZ;
  value.centerPoint = valueCenterPoint;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::point::GeoPoint valueEnuReferencePoint;
  ::ad::map::point::Longitude valueEnuReferencePointLongitude(-180);
  valueEnuReferencePoint.longitude = valueEnuReferencePointLongitude;
  ::ad::map::point::Latitude valueEnuReferencePointLatitude(-90);
  valueEnuReferencePoint.latitude = valueEnuReferencePointLatitude;
  ::ad::map::point::Altitude valueEnuReferencePointAltitude(-11000);
  valueEnuReferencePoint.altitude = valueEnuReferencePointAltitude;
  value.enuReferencePoint = valueEnuReferencePoint;
  ::ad::physics::Dimension3D valueDimension;
  ::ad::physics::Distance valueDimensionLength(-1e9);
  valueDimension.length = valueDimensionLength;
  ::ad::physics::Distance valueDimensionWidth(-1e9);
  valueDimension.width = valueDimensionWidth;
  ::ad::physics::Distance valueDimensionHeight(-1e9);
  valueDimension.height = valueDimensionHeight;
  value.dimension = valueDimension;

  // override member with data type value above input range maximum
  ::ad::map::point::ENUPoint invalidInitializedMember;
  ::ad::map::point::ENUCoordinate invalidInitializedMemberX(16384 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.centerPoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUObjectPositionValidInputRangeTests, testValidInputRangeHeadingTooSmall)
{
  ::ad::map::match::ENUObjectPosition value;
  ::ad::map::point::ENUPoint valueCenterPoint;
  ::ad::map::point::ENUCoordinate valueCenterPointX(-16384);
  valueCenterPoint.x = valueCenterPointX;
  ::ad::map::point::ENUCoordinate valueCenterPointY(-16384);
  valueCenterPoint.y = valueCenterPointY;
  ::ad::map::point::ENUCoordinate valueCenterPointZ(-16384);
  valueCenterPoint.z = valueCenterPointZ;
  value.centerPoint = valueCenterPoint;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::point::GeoPoint valueEnuReferencePoint;
  ::ad::map::point::Longitude valueEnuReferencePointLongitude(-180);
  valueEnuReferencePoint.longitude = valueEnuReferencePointLongitude;
  ::ad::map::point::Latitude valueEnuReferencePointLatitude(-90);
  valueEnuReferencePoint.latitude = valueEnuReferencePointLatitude;
  ::ad::map::point::Altitude valueEnuReferencePointAltitude(-11000);
  valueEnuReferencePoint.altitude = valueEnuReferencePointAltitude;
  value.enuReferencePoint = valueEnuReferencePoint;
  ::ad::physics::Dimension3D valueDimension;
  ::ad::physics::Distance valueDimensionLength(-1e9);
  valueDimension.length = valueDimensionLength;
  ::ad::physics::Distance valueDimensionWidth(-1e9);
  valueDimension.width = valueDimensionWidth;
  ::ad::physics::Distance valueDimensionHeight(-1e9);
  valueDimension.height = valueDimensionHeight;
  value.dimension = valueDimension;

  // override member with data type value below input range minimum
  ::ad::map::point::ENUHeading invalidInitializedMember(-3.141592655 * 1.1);
  value.heading = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUObjectPositionValidInputRangeTests, testValidInputRangeHeadingTooBig)
{
  ::ad::map::match::ENUObjectPosition value;
  ::ad::map::point::ENUPoint valueCenterPoint;
  ::ad::map::point::ENUCoordinate valueCenterPointX(-16384);
  valueCenterPoint.x = valueCenterPointX;
  ::ad::map::point::ENUCoordinate valueCenterPointY(-16384);
  valueCenterPoint.y = valueCenterPointY;
  ::ad::map::point::ENUCoordinate valueCenterPointZ(-16384);
  valueCenterPoint.z = valueCenterPointZ;
  value.centerPoint = valueCenterPoint;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::point::GeoPoint valueEnuReferencePoint;
  ::ad::map::point::Longitude valueEnuReferencePointLongitude(-180);
  valueEnuReferencePoint.longitude = valueEnuReferencePointLongitude;
  ::ad::map::point::Latitude valueEnuReferencePointLatitude(-90);
  valueEnuReferencePoint.latitude = valueEnuReferencePointLatitude;
  ::ad::map::point::Altitude valueEnuReferencePointAltitude(-11000);
  valueEnuReferencePoint.altitude = valueEnuReferencePointAltitude;
  value.enuReferencePoint = valueEnuReferencePoint;
  ::ad::physics::Dimension3D valueDimension;
  ::ad::physics::Distance valueDimensionLength(-1e9);
  valueDimension.length = valueDimensionLength;
  ::ad::physics::Distance valueDimensionWidth(-1e9);
  valueDimension.width = valueDimensionWidth;
  ::ad::physics::Distance valueDimensionHeight(-1e9);
  valueDimension.height = valueDimensionHeight;
  value.dimension = valueDimension;

  // override member with data type value above input range maximum
  ::ad::map::point::ENUHeading invalidInitializedMember(3.141592655 * 1.1);
  value.heading = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUObjectPositionValidInputRangeTests, testValidInputRangeheadingDefault)
{
  ::ad::map::match::ENUObjectPosition value;
  ::ad::map::point::ENUHeading valueDefault;
  value.heading = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUObjectPositionValidInputRangeTests, testValidInputRangeEnuReferencePointTooSmall)
{
  ::ad::map::match::ENUObjectPosition value;
  ::ad::map::point::ENUPoint valueCenterPoint;
  ::ad::map::point::ENUCoordinate valueCenterPointX(-16384);
  valueCenterPoint.x = valueCenterPointX;
  ::ad::map::point::ENUCoordinate valueCenterPointY(-16384);
  valueCenterPoint.y = valueCenterPointY;
  ::ad::map::point::ENUCoordinate valueCenterPointZ(-16384);
  valueCenterPoint.z = valueCenterPointZ;
  value.centerPoint = valueCenterPoint;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::point::GeoPoint valueEnuReferencePoint;
  ::ad::map::point::Longitude valueEnuReferencePointLongitude(-180);
  valueEnuReferencePoint.longitude = valueEnuReferencePointLongitude;
  ::ad::map::point::Latitude valueEnuReferencePointLatitude(-90);
  valueEnuReferencePoint.latitude = valueEnuReferencePointLatitude;
  ::ad::map::point::Altitude valueEnuReferencePointAltitude(-11000);
  valueEnuReferencePoint.altitude = valueEnuReferencePointAltitude;
  value.enuReferencePoint = valueEnuReferencePoint;
  ::ad::physics::Dimension3D valueDimension;
  ::ad::physics::Distance valueDimensionLength(-1e9);
  valueDimension.length = valueDimensionLength;
  ::ad::physics::Distance valueDimensionWidth(-1e9);
  valueDimension.width = valueDimensionWidth;
  ::ad::physics::Distance valueDimensionHeight(-1e9);
  valueDimension.height = valueDimensionHeight;
  value.dimension = valueDimension;

  // override member with data type value below input range minimum
  ::ad::map::point::GeoPoint invalidInitializedMember;
  ::ad::map::point::Longitude invalidInitializedMemberLongitude(-180 * 1.1);
  invalidInitializedMember.longitude = invalidInitializedMemberLongitude;
  value.enuReferencePoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUObjectPositionValidInputRangeTests, testValidInputRangeEnuReferencePointTooBig)
{
  ::ad::map::match::ENUObjectPosition value;
  ::ad::map::point::ENUPoint valueCenterPoint;
  ::ad::map::point::ENUCoordinate valueCenterPointX(-16384);
  valueCenterPoint.x = valueCenterPointX;
  ::ad::map::point::ENUCoordinate valueCenterPointY(-16384);
  valueCenterPoint.y = valueCenterPointY;
  ::ad::map::point::ENUCoordinate valueCenterPointZ(-16384);
  valueCenterPoint.z = valueCenterPointZ;
  value.centerPoint = valueCenterPoint;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::point::GeoPoint valueEnuReferencePoint;
  ::ad::map::point::Longitude valueEnuReferencePointLongitude(-180);
  valueEnuReferencePoint.longitude = valueEnuReferencePointLongitude;
  ::ad::map::point::Latitude valueEnuReferencePointLatitude(-90);
  valueEnuReferencePoint.latitude = valueEnuReferencePointLatitude;
  ::ad::map::point::Altitude valueEnuReferencePointAltitude(-11000);
  valueEnuReferencePoint.altitude = valueEnuReferencePointAltitude;
  value.enuReferencePoint = valueEnuReferencePoint;
  ::ad::physics::Dimension3D valueDimension;
  ::ad::physics::Distance valueDimensionLength(-1e9);
  valueDimension.length = valueDimensionLength;
  ::ad::physics::Distance valueDimensionWidth(-1e9);
  valueDimension.width = valueDimensionWidth;
  ::ad::physics::Distance valueDimensionHeight(-1e9);
  valueDimension.height = valueDimensionHeight;
  value.dimension = valueDimension;

  // override member with data type value above input range maximum
  ::ad::map::point::GeoPoint invalidInitializedMember;
  ::ad::map::point::Longitude invalidInitializedMemberLongitude(180 * 1.1);
  invalidInitializedMember.longitude = invalidInitializedMemberLongitude;
  value.enuReferencePoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUObjectPositionValidInputRangeTests, testValidInputRangeDimensionTooSmall)
{
  ::ad::map::match::ENUObjectPosition value;
  ::ad::map::point::ENUPoint valueCenterPoint;
  ::ad::map::point::ENUCoordinate valueCenterPointX(-16384);
  valueCenterPoint.x = valueCenterPointX;
  ::ad::map::point::ENUCoordinate valueCenterPointY(-16384);
  valueCenterPoint.y = valueCenterPointY;
  ::ad::map::point::ENUCoordinate valueCenterPointZ(-16384);
  valueCenterPoint.z = valueCenterPointZ;
  value.centerPoint = valueCenterPoint;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::point::GeoPoint valueEnuReferencePoint;
  ::ad::map::point::Longitude valueEnuReferencePointLongitude(-180);
  valueEnuReferencePoint.longitude = valueEnuReferencePointLongitude;
  ::ad::map::point::Latitude valueEnuReferencePointLatitude(-90);
  valueEnuReferencePoint.latitude = valueEnuReferencePointLatitude;
  ::ad::map::point::Altitude valueEnuReferencePointAltitude(-11000);
  valueEnuReferencePoint.altitude = valueEnuReferencePointAltitude;
  value.enuReferencePoint = valueEnuReferencePoint;
  ::ad::physics::Dimension3D valueDimension;
  ::ad::physics::Distance valueDimensionLength(-1e9);
  valueDimension.length = valueDimensionLength;
  ::ad::physics::Distance valueDimensionWidth(-1e9);
  valueDimension.width = valueDimensionWidth;
  ::ad::physics::Distance valueDimensionHeight(-1e9);
  valueDimension.height = valueDimensionHeight;
  value.dimension = valueDimension;

  // override member with data type value below input range minimum
  ::ad::physics::Dimension3D invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberLength(-1e9 * 1.1);
  invalidInitializedMember.length = invalidInitializedMemberLength;
  value.dimension = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUObjectPositionValidInputRangeTests, testValidInputRangeDimensionTooBig)
{
  ::ad::map::match::ENUObjectPosition value;
  ::ad::map::point::ENUPoint valueCenterPoint;
  ::ad::map::point::ENUCoordinate valueCenterPointX(-16384);
  valueCenterPoint.x = valueCenterPointX;
  ::ad::map::point::ENUCoordinate valueCenterPointY(-16384);
  valueCenterPoint.y = valueCenterPointY;
  ::ad::map::point::ENUCoordinate valueCenterPointZ(-16384);
  valueCenterPoint.z = valueCenterPointZ;
  value.centerPoint = valueCenterPoint;
  ::ad::map::point::ENUHeading valueHeading(-3.141592655);
  value.heading = valueHeading;
  ::ad::map::point::GeoPoint valueEnuReferencePoint;
  ::ad::map::point::Longitude valueEnuReferencePointLongitude(-180);
  valueEnuReferencePoint.longitude = valueEnuReferencePointLongitude;
  ::ad::map::point::Latitude valueEnuReferencePointLatitude(-90);
  valueEnuReferencePoint.latitude = valueEnuReferencePointLatitude;
  ::ad::map::point::Altitude valueEnuReferencePointAltitude(-11000);
  valueEnuReferencePoint.altitude = valueEnuReferencePointAltitude;
  value.enuReferencePoint = valueEnuReferencePoint;
  ::ad::physics::Dimension3D valueDimension;
  ::ad::physics::Distance valueDimensionLength(-1e9);
  valueDimension.length = valueDimensionLength;
  ::ad::physics::Distance valueDimensionWidth(-1e9);
  valueDimension.width = valueDimensionWidth;
  ::ad::physics::Distance valueDimensionHeight(-1e9);
  valueDimension.height = valueDimensionHeight;
  value.dimension = valueDimension;

  // override member with data type value above input range maximum
  ::ad::physics::Dimension3D invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberLength(1e9 * 1.1);
  invalidInitializedMember.length = invalidInitializedMemberLength;
  value.dimension = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
