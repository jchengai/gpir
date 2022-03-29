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

#include <gtest/gtest.h>

#include <limits>

#include "ad/map/restriction/VehicleDescriptorValidInputRange.hpp"

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRange)
{
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
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeTypeTooSmall)
{
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

  // override member with data type value below input range minimum
  ::ad::map::restriction::RoadUserType invalidInitializedMember(static_cast<::ad::map::restriction::RoadUserType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeTypeTooBig)
{
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

  // override member with data type value above input range maximum
  ::ad::map::restriction::RoadUserType invalidInitializedMember(static_cast<::ad::map::restriction::RoadUserType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeWidthTooSmall)
{
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

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.width = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeWidthTooBig)
{
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

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.width = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangewidthDefault)
{
  ::ad::map::restriction::VehicleDescriptor value;
  ::ad::physics::Distance valueDefault;
  value.width = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeHeightTooSmall)
{
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

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.height = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeHeightTooBig)
{
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

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.height = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeheightDefault)
{
  ::ad::map::restriction::VehicleDescriptor value;
  ::ad::physics::Distance valueDefault;
  value.height = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeLengthTooSmall)
{
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

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.length = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeLengthTooBig)
{
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

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.length = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangelengthDefault)
{
  ::ad::map::restriction::VehicleDescriptor value;
  ::ad::physics::Distance valueDefault;
  value.length = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeWeightTooSmall)
{
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

  // override member with data type value below input range minimum
  ::ad::physics::Weight invalidInitializedMember{}; // TODO: not invalid
  value.weight = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeWeightTooBig)
{
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

  // override member with data type value above input range maximum
  ::ad::physics::Weight invalidInitializedMember(48600.0 * 1.1);
  value.weight = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(VehicleDescriptorValidInputRangeTests, testValidInputRangeweightDefault)
{
  ::ad::map::restriction::VehicleDescriptor value;
  ::ad::physics::Weight valueDefault;
  value.weight = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
