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
#include "ad/map/access/MapMetaData.hpp"

class MapMetaDataTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::access::MapMetaData value;
    ::ad::map::access::TrafficType valueTrafficType(::ad::map::access::TrafficType::INVALID);
    value.trafficType = valueTrafficType;
    mValue = value;
  }

  ::ad::map::access::MapMetaData mValue;
};

TEST_F(MapMetaDataTests, copyConstruction)
{
  ::ad::map::access::MapMetaData value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMetaDataTests, moveConstruction)
{
  ::ad::map::access::MapMetaData tmpValue(mValue);
  ::ad::map::access::MapMetaData value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMetaDataTests, copyAssignment)
{
  ::ad::map::access::MapMetaData value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMetaDataTests, moveAssignment)
{
  ::ad::map::access::MapMetaData tmpValue(mValue);
  ::ad::map::access::MapMetaData value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMetaDataTests, comparisonOperatorEqual)
{
  ::ad::map::access::MapMetaData valueA = mValue;
  ::ad::map::access::MapMetaData valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(MapMetaDataTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(MapMetaDataTests, comparisonOperatorTrafficTypeDiffers)
{
  ::ad::map::access::MapMetaData valueA = mValue;
  ::ad::map::access::TrafficType trafficType(::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC);
  valueA.trafficType = trafficType;
  ::ad::map::access::MapMetaData valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
