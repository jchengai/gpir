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
#include "ad/map/point/ENUEdgeCache.hpp"

class ENUEdgeCacheTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::point::ENUEdgeCache value;
    ::ad::map::point::ENUEdge valueEnuEdge;
    ::ad::map::point::ENUPoint valueEnuEdgeElement;
    ::ad::map::point::ENUCoordinate valueEnuEdgeElementX(-16384);
    valueEnuEdgeElement.x = valueEnuEdgeElementX;
    ::ad::map::point::ENUCoordinate valueEnuEdgeElementY(-16384);
    valueEnuEdgeElement.y = valueEnuEdgeElementY;
    ::ad::map::point::ENUCoordinate valueEnuEdgeElementZ(-16384);
    valueEnuEdgeElement.z = valueEnuEdgeElementZ;
    valueEnuEdge.resize(1, valueEnuEdgeElement);
    value.enuEdge = valueEnuEdge;
    uint64_t valueEnuVersion{std::numeric_limits<uint64_t>::min()};
    value.enuVersion = valueEnuVersion;
    mValue = value;
  }

  ::ad::map::point::ENUEdgeCache mValue;
};

TEST_F(ENUEdgeCacheTests, copyConstruction)
{
  ::ad::map::point::ENUEdgeCache value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUEdgeCacheTests, moveConstruction)
{
  ::ad::map::point::ENUEdgeCache tmpValue(mValue);
  ::ad::map::point::ENUEdgeCache value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUEdgeCacheTests, copyAssignment)
{
  ::ad::map::point::ENUEdgeCache value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUEdgeCacheTests, moveAssignment)
{
  ::ad::map::point::ENUEdgeCache tmpValue(mValue);
  ::ad::map::point::ENUEdgeCache value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ENUEdgeCacheTests, comparisonOperatorEqual)
{
  ::ad::map::point::ENUEdgeCache valueA = mValue;
  ::ad::map::point::ENUEdgeCache valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ENUEdgeCacheTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ENUEdgeCacheTests, comparisonOperatorEnuEdgeDiffers)
{
  ::ad::map::point::ENUEdgeCache valueA = mValue;
  ::ad::map::point::ENUEdge enuEdge;
  ::ad::map::point::ENUPoint enuEdgeElement;
  ::ad::map::point::ENUCoordinate enuEdgeElementX(16384);
  enuEdgeElement.x = enuEdgeElementX;
  ::ad::map::point::ENUCoordinate enuEdgeElementY(16384);
  enuEdgeElement.y = enuEdgeElementY;
  ::ad::map::point::ENUCoordinate enuEdgeElementZ(16384);
  enuEdgeElement.z = enuEdgeElementZ;
  enuEdge.resize(2, enuEdgeElement);
  valueA.enuEdge = enuEdge;
  ::ad::map::point::ENUEdgeCache valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ENUEdgeCacheTests, comparisonOperatorEnuVersionDiffers)
{
  ::ad::map::point::ENUEdgeCache valueA = mValue;
  uint64_t enuVersion{std::numeric_limits<uint64_t>::max()};
  valueA.enuVersion = enuVersion;
  ::ad::map::point::ENUEdgeCache valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
