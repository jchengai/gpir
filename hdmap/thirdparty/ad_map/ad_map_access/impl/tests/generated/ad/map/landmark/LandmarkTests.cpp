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

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wself-assign-overloaded"
#endif

#include <gtest/gtest.h>
#include <limits>
#include "ad/map/landmark/Landmark.hpp"

class LandmarkTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::landmark::Landmark value;
    ::ad::map::landmark::LandmarkId valueId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
    value.id = valueId;
    ::ad::map::landmark::LandmarkType valueType(::ad::map::landmark::LandmarkType::INVALID);
    value.type = valueType;
    ::ad::map::point::ECEFPoint valuePosition;
    ::ad::map::point::ECEFCoordinate valuePositionX(-6400000);
    valuePosition.x = valuePositionX;
    ::ad::map::point::ECEFCoordinate valuePositionY(-6400000);
    valuePosition.y = valuePositionY;
    ::ad::map::point::ECEFCoordinate valuePositionZ(-6400000);
    valuePosition.z = valuePositionZ;
    value.position = valuePosition;
    ::ad::map::point::ECEFPoint valueOrientation;
    ::ad::map::point::ECEFCoordinate valueOrientationX(-6400000);
    valueOrientation.x = valueOrientationX;
    ::ad::map::point::ECEFCoordinate valueOrientationY(-6400000);
    valueOrientation.y = valueOrientationY;
    ::ad::map::point::ECEFCoordinate valueOrientationZ(-6400000);
    valueOrientation.z = valueOrientationZ;
    value.orientation = valueOrientation;
    ::ad::map::point::Geometry valueBoundingBox;
    bool valueBoundingBoxIsValid{true};
    valueBoundingBox.isValid = valueBoundingBoxIsValid;
    bool valueBoundingBoxIsClosed{true};
    valueBoundingBox.isClosed = valueBoundingBoxIsClosed;
    ::ad::map::point::ECEFEdge valueBoundingBoxEcefEdge;
    ::ad::map::point::ECEFPoint valueBoundingBoxEcefEdgeElement;
    ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementX(-6400000);
    valueBoundingBoxEcefEdgeElement.x = valueBoundingBoxEcefEdgeElementX;
    ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementY(-6400000);
    valueBoundingBoxEcefEdgeElement.y = valueBoundingBoxEcefEdgeElementY;
    ::ad::map::point::ECEFCoordinate valueBoundingBoxEcefEdgeElementZ(-6400000);
    valueBoundingBoxEcefEdgeElement.z = valueBoundingBoxEcefEdgeElementZ;
    valueBoundingBoxEcefEdge.resize(1, valueBoundingBoxEcefEdgeElement);
    valueBoundingBox.ecefEdge = valueBoundingBoxEcefEdge;
    ::ad::physics::Distance valueBoundingBoxLength(-1e9);
    valueBoundingBox.length = valueBoundingBoxLength;
    ::ad::map::point::ENUEdgeCache valueBoundingBoxPrivate_enuEdgeCache;
    ::ad::map::point::ENUEdge valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
    ::ad::map::point::ENUPoint valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
    ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
    valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
    ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
    valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
    ::ad::map::point::ENUCoordinate valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
    valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
    valueBoundingBoxPrivate_enuEdgeCacheEnuEdge.resize(1, valueBoundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
    valueBoundingBoxPrivate_enuEdgeCache.enuEdge = valueBoundingBoxPrivate_enuEdgeCacheEnuEdge;
    uint64_t valueBoundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
    valueBoundingBoxPrivate_enuEdgeCache.enuVersion = valueBoundingBoxPrivate_enuEdgeCacheEnuVersion;
    valueBoundingBox.private_enuEdgeCache = valueBoundingBoxPrivate_enuEdgeCache;
    value.boundingBox = valueBoundingBox;
    std::string valueSupplementaryText{"min"};
    value.supplementaryText = valueSupplementaryText;
    ::ad::map::landmark::TrafficLightType valueTrafficLightType(::ad::map::landmark::TrafficLightType::INVALID);
    value.trafficLightType = valueTrafficLightType;
    ::ad::map::landmark::TrafficSignType valueTrafficSignType(::ad::map::landmark::TrafficSignType::INVALID);
    value.trafficSignType = valueTrafficSignType;
    mValue = value;
  }

  ::ad::map::landmark::Landmark mValue;
};

TEST_F(LandmarkTests, copyConstruction)
{
  ::ad::map::landmark::Landmark value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LandmarkTests, moveConstruction)
{
  ::ad::map::landmark::Landmark tmpValue(mValue);
  ::ad::map::landmark::Landmark value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(LandmarkTests, copyAssignment)
{
  ::ad::map::landmark::Landmark value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(LandmarkTests, moveAssignment)
{
  ::ad::map::landmark::Landmark tmpValue(mValue);
  ::ad::map::landmark::Landmark value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LandmarkTests, comparisonOperatorEqual)
{
  ::ad::map::landmark::Landmark valueA = mValue;
  ::ad::map::landmark::Landmark valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(LandmarkTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(LandmarkTests, comparisonOperatorIdDiffers)
{
  ::ad::map::landmark::Landmark valueA = mValue;
  ::ad::map::landmark::LandmarkId id(std::numeric_limits<::ad::map::landmark::LandmarkId>::max());
  valueA.id = id;
  ::ad::map::landmark::Landmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LandmarkTests, comparisonOperatorTypeDiffers)
{
  ::ad::map::landmark::Landmark valueA = mValue;
  ::ad::map::landmark::LandmarkType type(::ad::map::landmark::LandmarkType::OTHER);
  valueA.type = type;
  ::ad::map::landmark::Landmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LandmarkTests, comparisonOperatorPositionDiffers)
{
  ::ad::map::landmark::Landmark valueA = mValue;
  ::ad::map::point::ECEFPoint position;
  ::ad::map::point::ECEFCoordinate positionX(6400000);
  position.x = positionX;
  ::ad::map::point::ECEFCoordinate positionY(6400000);
  position.y = positionY;
  ::ad::map::point::ECEFCoordinate positionZ(6400000);
  position.z = positionZ;
  valueA.position = position;
  ::ad::map::landmark::Landmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LandmarkTests, comparisonOperatorOrientationDiffers)
{
  ::ad::map::landmark::Landmark valueA = mValue;
  ::ad::map::point::ECEFPoint orientation;
  ::ad::map::point::ECEFCoordinate orientationX(6400000);
  orientation.x = orientationX;
  ::ad::map::point::ECEFCoordinate orientationY(6400000);
  orientation.y = orientationY;
  ::ad::map::point::ECEFCoordinate orientationZ(6400000);
  orientation.z = orientationZ;
  valueA.orientation = orientation;
  ::ad::map::landmark::Landmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LandmarkTests, comparisonOperatorBoundingBoxDiffers)
{
  ::ad::map::landmark::Landmark valueA = mValue;
  ::ad::map::point::Geometry boundingBox;
  bool boundingBoxIsValid{false};
  boundingBox.isValid = boundingBoxIsValid;
  bool boundingBoxIsClosed{false};
  boundingBox.isClosed = boundingBoxIsClosed;
  ::ad::map::point::ECEFEdge boundingBoxEcefEdge;
  ::ad::map::point::ECEFPoint boundingBoxEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate boundingBoxEcefEdgeElementX(6400000);
  boundingBoxEcefEdgeElement.x = boundingBoxEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate boundingBoxEcefEdgeElementY(6400000);
  boundingBoxEcefEdgeElement.y = boundingBoxEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate boundingBoxEcefEdgeElementZ(6400000);
  boundingBoxEcefEdgeElement.z = boundingBoxEcefEdgeElementZ;
  boundingBoxEcefEdge.resize(2, boundingBoxEcefEdgeElement);
  boundingBox.ecefEdge = boundingBoxEcefEdge;
  ::ad::physics::Distance boundingBoxLength(1e9);
  boundingBox.length = boundingBoxLength;
  ::ad::map::point::ENUEdgeCache boundingBoxPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge boundingBoxPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint boundingBoxPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate boundingBoxPrivate_enuEdgeCacheEnuEdgeElementX(16384);
  boundingBoxPrivate_enuEdgeCacheEnuEdgeElement.x = boundingBoxPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate boundingBoxPrivate_enuEdgeCacheEnuEdgeElementY(16384);
  boundingBoxPrivate_enuEdgeCacheEnuEdgeElement.y = boundingBoxPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate boundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ(16384);
  boundingBoxPrivate_enuEdgeCacheEnuEdgeElement.z = boundingBoxPrivate_enuEdgeCacheEnuEdgeElementZ;
  boundingBoxPrivate_enuEdgeCacheEnuEdge.resize(2, boundingBoxPrivate_enuEdgeCacheEnuEdgeElement);
  boundingBoxPrivate_enuEdgeCache.enuEdge = boundingBoxPrivate_enuEdgeCacheEnuEdge;
  uint64_t boundingBoxPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::max()};
  boundingBoxPrivate_enuEdgeCache.enuVersion = boundingBoxPrivate_enuEdgeCacheEnuVersion;
  boundingBox.private_enuEdgeCache = boundingBoxPrivate_enuEdgeCache;
  valueA.boundingBox = boundingBox;
  ::ad::map::landmark::Landmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LandmarkTests, comparisonOperatorSupplementaryTextDiffers)
{
  ::ad::map::landmark::Landmark valueA = mValue;
  std::string supplementaryText{"max"};
  valueA.supplementaryText = supplementaryText;
  ::ad::map::landmark::Landmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LandmarkTests, comparisonOperatorTrafficLightTypeDiffers)
{
  ::ad::map::landmark::Landmark valueA = mValue;
  ::ad::map::landmark::TrafficLightType trafficLightType(
    ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN);
  valueA.trafficLightType = trafficLightType;
  ::ad::map::landmark::Landmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LandmarkTests, comparisonOperatorTrafficSignTypeDiffers)
{
  ::ad::map::landmark::Landmark valueA = mValue;
  ::ad::map::landmark::TrafficSignType trafficSignType(::ad::map::landmark::TrafficSignType::UNKNOWN);
  valueA.trafficSignType = trafficSignType;
  ::ad::map::landmark::Landmark valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
