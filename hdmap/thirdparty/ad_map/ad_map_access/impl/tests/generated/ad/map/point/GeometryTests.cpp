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
#include "ad/map/point/Geometry.hpp"

class GeometryTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::point::Geometry value;
    bool valueIsValid{true};
    value.isValid = valueIsValid;
    bool valueIsClosed{true};
    value.isClosed = valueIsClosed;
    ::ad::map::point::ECEFEdge valueEcefEdge;
    ::ad::map::point::ECEFPoint valueEcefEdgeElement;
    ::ad::map::point::ECEFCoordinate valueEcefEdgeElementX(-6400000);
    valueEcefEdgeElement.x = valueEcefEdgeElementX;
    ::ad::map::point::ECEFCoordinate valueEcefEdgeElementY(-6400000);
    valueEcefEdgeElement.y = valueEcefEdgeElementY;
    ::ad::map::point::ECEFCoordinate valueEcefEdgeElementZ(-6400000);
    valueEcefEdgeElement.z = valueEcefEdgeElementZ;
    valueEcefEdge.resize(1, valueEcefEdgeElement);
    value.ecefEdge = valueEcefEdge;
    ::ad::physics::Distance valueLength(-1e9);
    value.length = valueLength;
    ::ad::map::point::ENUEdgeCache valuePrivate_enuEdgeCache;
    ::ad::map::point::ENUEdge valuePrivate_enuEdgeCacheEnuEdge;
    ::ad::map::point::ENUPoint valuePrivate_enuEdgeCacheEnuEdgeElement;
    ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementX(-16384);
    valuePrivate_enuEdgeCacheEnuEdgeElement.x = valuePrivate_enuEdgeCacheEnuEdgeElementX;
    ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementY(-16384);
    valuePrivate_enuEdgeCacheEnuEdgeElement.y = valuePrivate_enuEdgeCacheEnuEdgeElementY;
    ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
    valuePrivate_enuEdgeCacheEnuEdgeElement.z = valuePrivate_enuEdgeCacheEnuEdgeElementZ;
    valuePrivate_enuEdgeCacheEnuEdge.resize(1, valuePrivate_enuEdgeCacheEnuEdgeElement);
    valuePrivate_enuEdgeCache.enuEdge = valuePrivate_enuEdgeCacheEnuEdge;
    uint64_t valuePrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
    valuePrivate_enuEdgeCache.enuVersion = valuePrivate_enuEdgeCacheEnuVersion;
    value.private_enuEdgeCache = valuePrivate_enuEdgeCache;
    mValue = value;
  }

  ::ad::map::point::Geometry mValue;
};

TEST_F(GeometryTests, copyConstruction)
{
  ::ad::map::point::Geometry value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(GeometryTests, moveConstruction)
{
  ::ad::map::point::Geometry tmpValue(mValue);
  ::ad::map::point::Geometry value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(GeometryTests, copyAssignment)
{
  ::ad::map::point::Geometry value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(GeometryTests, moveAssignment)
{
  ::ad::map::point::Geometry tmpValue(mValue);
  ::ad::map::point::Geometry value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(GeometryTests, comparisonOperatorEqual)
{
  ::ad::map::point::Geometry valueA = mValue;
  ::ad::map::point::Geometry valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(GeometryTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(GeometryTests, comparisonOperatorIsValidDiffers)
{
  ::ad::map::point::Geometry valueA = mValue;
  bool isValid{false};
  valueA.isValid = isValid;
  ::ad::map::point::Geometry valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(GeometryTests, comparisonOperatorIsClosedDiffers)
{
  ::ad::map::point::Geometry valueA = mValue;
  bool isClosed{false};
  valueA.isClosed = isClosed;
  ::ad::map::point::Geometry valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(GeometryTests, comparisonOperatorEcefEdgeDiffers)
{
  ::ad::map::point::Geometry valueA = mValue;
  ::ad::map::point::ECEFEdge ecefEdge;
  ::ad::map::point::ECEFPoint ecefEdgeElement;
  ::ad::map::point::ECEFCoordinate ecefEdgeElementX(6400000);
  ecefEdgeElement.x = ecefEdgeElementX;
  ::ad::map::point::ECEFCoordinate ecefEdgeElementY(6400000);
  ecefEdgeElement.y = ecefEdgeElementY;
  ::ad::map::point::ECEFCoordinate ecefEdgeElementZ(6400000);
  ecefEdgeElement.z = ecefEdgeElementZ;
  ecefEdge.resize(2, ecefEdgeElement);
  valueA.ecefEdge = ecefEdge;
  ::ad::map::point::Geometry valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(GeometryTests, comparisonOperatorLengthDiffers)
{
  ::ad::map::point::Geometry valueA = mValue;
  ::ad::physics::Distance length(1e9);
  valueA.length = length;
  ::ad::map::point::Geometry valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(GeometryTests, comparisonOperatorPrivate_enuEdgeCacheDiffers)
{
  ::ad::map::point::Geometry valueA = mValue;
  ::ad::map::point::ENUEdgeCache private_enuEdgeCache;
  ::ad::map::point::ENUEdge private_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint private_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate private_enuEdgeCacheEnuEdgeElementX(16384);
  private_enuEdgeCacheEnuEdgeElement.x = private_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate private_enuEdgeCacheEnuEdgeElementY(16384);
  private_enuEdgeCacheEnuEdgeElement.y = private_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate private_enuEdgeCacheEnuEdgeElementZ(16384);
  private_enuEdgeCacheEnuEdgeElement.z = private_enuEdgeCacheEnuEdgeElementZ;
  private_enuEdgeCacheEnuEdge.resize(2, private_enuEdgeCacheEnuEdgeElement);
  private_enuEdgeCache.enuEdge = private_enuEdgeCacheEnuEdge;
  uint64_t private_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::max()};
  private_enuEdgeCache.enuVersion = private_enuEdgeCacheEnuVersion;
  valueA.private_enuEdgeCache = private_enuEdgeCache;
  ::ad::map::point::Geometry valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
