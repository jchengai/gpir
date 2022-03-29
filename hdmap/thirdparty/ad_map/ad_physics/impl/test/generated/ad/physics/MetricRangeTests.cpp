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
#include "ad/physics/MetricRange.hpp"

class MetricRangeTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::physics::MetricRange value;
    ::ad::physics::Distance valueMinimum(-1e9);
    valueMinimum = ::ad::physics::Distance(0.); // set to valid value within struct
    value.minimum = valueMinimum;
    ::ad::physics::Distance valueMaximum(-1e9);
    value.maximum = valueMaximum;
    value.maximum = value.minimum;
    value.minimum = value.maximum;
    mValue = value;
  }

  ::ad::physics::MetricRange mValue;
};

TEST_F(MetricRangeTests, copyConstruction)
{
  ::ad::physics::MetricRange value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(MetricRangeTests, moveConstruction)
{
  ::ad::physics::MetricRange tmpValue(mValue);
  ::ad::physics::MetricRange value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(MetricRangeTests, copyAssignment)
{
  ::ad::physics::MetricRange value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(MetricRangeTests, moveAssignment)
{
  ::ad::physics::MetricRange tmpValue(mValue);
  ::ad::physics::MetricRange value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(MetricRangeTests, comparisonOperatorEqual)
{
  ::ad::physics::MetricRange valueA = mValue;
  ::ad::physics::MetricRange valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(MetricRangeTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(MetricRangeTests, comparisonOperatorMinimumDiffers)
{
  ::ad::physics::MetricRange valueA = mValue;
  ::ad::physics::Distance minimum(1e9);
  valueA.minimum = minimum;
  ::ad::physics::MetricRange valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MetricRangeTests, comparisonOperatorMaximumDiffers)
{
  ::ad::physics::MetricRange valueA = mValue;
  ::ad::physics::Distance maximum(1e9);
  valueA.maximum = maximum;
  ::ad::physics::MetricRange valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
