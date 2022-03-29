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
#include "ad/map/restriction/VehicleDescriptor.hpp"

class VehicleDescriptorTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::restriction::VehicleDescriptor value;
    ::ad::map::restriction::PassengerCount valuePassengers(
      std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
    value.passengers = valuePassengers;
    ::ad::map::restriction::RoadUserType valueType(::ad::map::restriction::RoadUserType::INVALID);
    value.type = valueType;
    ::ad::physics::Distance valueWidth(-1e9);
    value.width = valueWidth;
    ::ad::physics::Distance valueHeight(-1e9);
    value.height = valueHeight;
    ::ad::physics::Distance valueLength(-1e9);
    value.length = valueLength;
    ::ad::physics::Weight valueWeight(std::numeric_limits<::ad::physics::Weight>::lowest());
    value.weight = valueWeight;
    mValue = value;
  }

  ::ad::map::restriction::VehicleDescriptor mValue;
};

TEST_F(VehicleDescriptorTests, copyConstruction)
{
  ::ad::map::restriction::VehicleDescriptor value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(VehicleDescriptorTests, moveConstruction)
{
  ::ad::map::restriction::VehicleDescriptor tmpValue(mValue);
  ::ad::map::restriction::VehicleDescriptor value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(VehicleDescriptorTests, copyAssignment)
{
  ::ad::map::restriction::VehicleDescriptor value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(VehicleDescriptorTests, moveAssignment)
{
  ::ad::map::restriction::VehicleDescriptor tmpValue(mValue);
  ::ad::map::restriction::VehicleDescriptor value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(VehicleDescriptorTests, comparisonOperatorEqual)
{
  ::ad::map::restriction::VehicleDescriptor valueA = mValue;
  ::ad::map::restriction::VehicleDescriptor valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(VehicleDescriptorTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(VehicleDescriptorTests, comparisonOperatorPassengersDiffers)
{
  ::ad::map::restriction::VehicleDescriptor valueA = mValue;
  ::ad::map::restriction::PassengerCount passengers(std::numeric_limits<::ad::map::restriction::PassengerCount>::max());
  valueA.passengers = passengers;
  ::ad::map::restriction::VehicleDescriptor valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(VehicleDescriptorTests, comparisonOperatorTypeDiffers)
{
  ::ad::map::restriction::VehicleDescriptor valueA = mValue;
  ::ad::map::restriction::RoadUserType type(::ad::map::restriction::RoadUserType::CAR_DIESEL);
  valueA.type = type;
  ::ad::map::restriction::VehicleDescriptor valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(VehicleDescriptorTests, comparisonOperatorWidthDiffers)
{
  ::ad::map::restriction::VehicleDescriptor valueA = mValue;
  ::ad::physics::Distance width(1e9);
  valueA.width = width;
  ::ad::map::restriction::VehicleDescriptor valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(VehicleDescriptorTests, comparisonOperatorHeightDiffers)
{
  ::ad::map::restriction::VehicleDescriptor valueA = mValue;
  ::ad::physics::Distance height(1e9);
  valueA.height = height;
  ::ad::map::restriction::VehicleDescriptor valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(VehicleDescriptorTests, comparisonOperatorLengthDiffers)
{
  ::ad::map::restriction::VehicleDescriptor valueA = mValue;
  ::ad::physics::Distance length(1e9);
  valueA.length = length;
  ::ad::map::restriction::VehicleDescriptor valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(VehicleDescriptorTests, comparisonOperatorWeightDiffers)
{
  ::ad::map::restriction::VehicleDescriptor valueA = mValue;
  ::ad::physics::Weight weight(48600.0);
  valueA.weight = weight;
  ::ad::map::restriction::VehicleDescriptor valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
