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
#include "ad/map/point/BoundingSphere.hpp"

class BoundingSphereTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::point::BoundingSphere value;
    ::ad::map::point::ECEFPoint valueCenter;
    ::ad::map::point::ECEFCoordinate valueCenterX(-6400000);
    valueCenter.x = valueCenterX;
    ::ad::map::point::ECEFCoordinate valueCenterY(-6400000);
    valueCenter.y = valueCenterY;
    ::ad::map::point::ECEFCoordinate valueCenterZ(-6400000);
    valueCenter.z = valueCenterZ;
    value.center = valueCenter;
    ::ad::physics::Distance valueRadius(-1e9);
    value.radius = valueRadius;
    mValue = value;
  }

  ::ad::map::point::BoundingSphere mValue;
};

TEST_F(BoundingSphereTests, copyConstruction)
{
  ::ad::map::point::BoundingSphere value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(BoundingSphereTests, moveConstruction)
{
  ::ad::map::point::BoundingSphere tmpValue(mValue);
  ::ad::map::point::BoundingSphere value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(BoundingSphereTests, copyAssignment)
{
  ::ad::map::point::BoundingSphere value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(BoundingSphereTests, moveAssignment)
{
  ::ad::map::point::BoundingSphere tmpValue(mValue);
  ::ad::map::point::BoundingSphere value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(BoundingSphereTests, comparisonOperatorEqual)
{
  ::ad::map::point::BoundingSphere valueA = mValue;
  ::ad::map::point::BoundingSphere valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(BoundingSphereTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(BoundingSphereTests, comparisonOperatorCenterDiffers)
{
  ::ad::map::point::BoundingSphere valueA = mValue;
  ::ad::map::point::ECEFPoint center;
  ::ad::map::point::ECEFCoordinate centerX(6400000);
  center.x = centerX;
  ::ad::map::point::ECEFCoordinate centerY(6400000);
  center.y = centerY;
  ::ad::map::point::ECEFCoordinate centerZ(6400000);
  center.z = centerZ;
  valueA.center = center;
  ::ad::map::point::BoundingSphere valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(BoundingSphereTests, comparisonOperatorRadiusDiffers)
{
  ::ad::map::point::BoundingSphere valueA = mValue;
  ::ad::physics::Distance radius(1e9);
  valueA.radius = radius;
  ::ad::map::point::BoundingSphere valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
