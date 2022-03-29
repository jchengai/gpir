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
#include "ad/physics/ParametricRange.hpp"

class ParametricRangeTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::physics::ParametricRange value;
    ::ad::physics::ParametricValue valueMinimum(0.);
    value.minimum = valueMinimum;
    ::ad::physics::ParametricValue valueMaximum(0.);
    value.maximum = valueMaximum;
    value.maximum = value.minimum;
    value.minimum = value.maximum;
    mValue = value;
  }

  ::ad::physics::ParametricRange mValue;
};

TEST_F(ParametricRangeTests, copyConstruction)
{
  ::ad::physics::ParametricRange value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ParametricRangeTests, moveConstruction)
{
  ::ad::physics::ParametricRange tmpValue(mValue);
  ::ad::physics::ParametricRange value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ParametricRangeTests, copyAssignment)
{
  ::ad::physics::ParametricRange value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ParametricRangeTests, moveAssignment)
{
  ::ad::physics::ParametricRange tmpValue(mValue);
  ::ad::physics::ParametricRange value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ParametricRangeTests, comparisonOperatorEqual)
{
  ::ad::physics::ParametricRange valueA = mValue;
  ::ad::physics::ParametricRange valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ParametricRangeTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ParametricRangeTests, comparisonOperatorMinimumDiffers)
{
  ::ad::physics::ParametricRange valueA = mValue;
  ::ad::physics::ParametricValue minimum(1.);
  valueA.minimum = minimum;
  ::ad::physics::ParametricRange valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ParametricRangeTests, comparisonOperatorMaximumDiffers)
{
  ::ad::physics::ParametricRange valueA = mValue;
  ::ad::physics::ParametricValue maximum(1.);
  valueA.maximum = maximum;
  ::ad::physics::ParametricRange valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
