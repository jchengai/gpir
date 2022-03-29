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
#include "ad/physics/AngleRange.hpp"

class AngleRangeTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::physics::AngleRange value;
    ::ad::physics::Angle valueMinimum(-6.283185308);
    value.minimum = valueMinimum;
    ::ad::physics::Angle valueMaximum(-6.283185308);
    value.maximum = valueMaximum;
    mValue = value;
  }

  ::ad::physics::AngleRange mValue;
};

TEST_F(AngleRangeTests, copyConstruction)
{
  ::ad::physics::AngleRange value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(AngleRangeTests, moveConstruction)
{
  ::ad::physics::AngleRange tmpValue(mValue);
  ::ad::physics::AngleRange value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(AngleRangeTests, copyAssignment)
{
  ::ad::physics::AngleRange value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(AngleRangeTests, moveAssignment)
{
  ::ad::physics::AngleRange tmpValue(mValue);
  ::ad::physics::AngleRange value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(AngleRangeTests, comparisonOperatorEqual)
{
  ::ad::physics::AngleRange valueA = mValue;
  ::ad::physics::AngleRange valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(AngleRangeTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(AngleRangeTests, comparisonOperatorMinimumDiffers)
{
  ::ad::physics::AngleRange valueA = mValue;
  ::ad::physics::Angle minimum(6.283185308);
  valueA.minimum = minimum;
  ::ad::physics::AngleRange valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(AngleRangeTests, comparisonOperatorMaximumDiffers)
{
  ::ad::physics::AngleRange valueA = mValue;
  ::ad::physics::Angle maximum(6.283185308);
  valueA.maximum = maximum;
  ::ad::physics::AngleRange valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
