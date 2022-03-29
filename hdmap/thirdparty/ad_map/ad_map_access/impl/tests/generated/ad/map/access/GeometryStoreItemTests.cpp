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
#include "ad/map/access/GeometryStoreItem.hpp"

class GeometryStoreItemTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::access::GeometryStoreItem value;
    uint32_t valueLeftEdgeOffset{std::numeric_limits<uint32_t>::min()};
    value.leftEdgeOffset = valueLeftEdgeOffset;
    uint32_t valueRightEdgeOffset{std::numeric_limits<uint32_t>::min()};
    value.rightEdgeOffset = valueRightEdgeOffset;
    uint32_t valueLeftEdgePoints{std::numeric_limits<uint32_t>::min()};
    value.leftEdgePoints = valueLeftEdgePoints;
    uint32_t valueRightEdgePoints{std::numeric_limits<uint32_t>::min()};
    value.rightEdgePoints = valueRightEdgePoints;
    mValue = value;
  }

  ::ad::map::access::GeometryStoreItem mValue;
};

TEST_F(GeometryStoreItemTests, copyConstruction)
{
  ::ad::map::access::GeometryStoreItem value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(GeometryStoreItemTests, moveConstruction)
{
  ::ad::map::access::GeometryStoreItem tmpValue(mValue);
  ::ad::map::access::GeometryStoreItem value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(GeometryStoreItemTests, copyAssignment)
{
  ::ad::map::access::GeometryStoreItem value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(GeometryStoreItemTests, moveAssignment)
{
  ::ad::map::access::GeometryStoreItem tmpValue(mValue);
  ::ad::map::access::GeometryStoreItem value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(GeometryStoreItemTests, comparisonOperatorEqual)
{
  ::ad::map::access::GeometryStoreItem valueA = mValue;
  ::ad::map::access::GeometryStoreItem valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(GeometryStoreItemTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(GeometryStoreItemTests, comparisonOperatorLeftEdgeOffsetDiffers)
{
  ::ad::map::access::GeometryStoreItem valueA = mValue;
  uint32_t leftEdgeOffset{std::numeric_limits<uint32_t>::max()};
  valueA.leftEdgeOffset = leftEdgeOffset;
  ::ad::map::access::GeometryStoreItem valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(GeometryStoreItemTests, comparisonOperatorRightEdgeOffsetDiffers)
{
  ::ad::map::access::GeometryStoreItem valueA = mValue;
  uint32_t rightEdgeOffset{std::numeric_limits<uint32_t>::max()};
  valueA.rightEdgeOffset = rightEdgeOffset;
  ::ad::map::access::GeometryStoreItem valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(GeometryStoreItemTests, comparisonOperatorLeftEdgePointsDiffers)
{
  ::ad::map::access::GeometryStoreItem valueA = mValue;
  uint32_t leftEdgePoints{std::numeric_limits<uint32_t>::max()};
  valueA.leftEdgePoints = leftEdgePoints;
  ::ad::map::access::GeometryStoreItem valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(GeometryStoreItemTests, comparisonOperatorRightEdgePointsDiffers)
{
  ::ad::map::access::GeometryStoreItem valueA = mValue;
  uint32_t rightEdgePoints{std::numeric_limits<uint32_t>::max()};
  valueA.rightEdgePoints = rightEdgePoints;
  ::ad::map::access::GeometryStoreItem valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
