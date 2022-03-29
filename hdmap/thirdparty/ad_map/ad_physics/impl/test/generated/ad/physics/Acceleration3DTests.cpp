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
#include "ad/physics/Acceleration3D.hpp"

class Acceleration3DTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::physics::Acceleration3D value;
    ::ad::physics::Acceleration valueX(-1e2);
    value.x = valueX;
    ::ad::physics::Acceleration valueY(-1e2);
    value.y = valueY;
    ::ad::physics::Acceleration valueZ(-1e2);
    value.z = valueZ;
    mValue = value;
  }

  ::ad::physics::Acceleration3D mValue;
};

TEST_F(Acceleration3DTests, copyConstruction)
{
  ::ad::physics::Acceleration3D value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(Acceleration3DTests, moveConstruction)
{
  ::ad::physics::Acceleration3D tmpValue(mValue);
  ::ad::physics::Acceleration3D value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(Acceleration3DTests, copyAssignment)
{
  ::ad::physics::Acceleration3D value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(Acceleration3DTests, moveAssignment)
{
  ::ad::physics::Acceleration3D tmpValue(mValue);
  ::ad::physics::Acceleration3D value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(Acceleration3DTests, comparisonOperatorEqual)
{
  ::ad::physics::Acceleration3D valueA = mValue;
  ::ad::physics::Acceleration3D valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(Acceleration3DTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(Acceleration3DTests, comparisonOperatorXDiffers)
{
  ::ad::physics::Acceleration3D valueA = mValue;
  ::ad::physics::Acceleration x(1e2);
  valueA.x = x;
  ::ad::physics::Acceleration3D valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(Acceleration3DTests, comparisonOperatorYDiffers)
{
  ::ad::physics::Acceleration3D valueA = mValue;
  ::ad::physics::Acceleration y(1e2);
  valueA.y = y;
  ::ad::physics::Acceleration3D valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(Acceleration3DTests, comparisonOperatorZDiffers)
{
  ::ad::physics::Acceleration3D valueA = mValue;
  ::ad::physics::Acceleration z(1e2);
  valueA.z = z;
  ::ad::physics::Acceleration3D valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
