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
#include "ad/map/lane/ContactLane.hpp"

class ContactLaneTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::lane::ContactLane value;
    ::ad::map::lane::LaneId valueToLane(1);
    value.toLane = valueToLane;
    ::ad::map::lane::ContactLocation valueLocation(::ad::map::lane::ContactLocation::INVALID);
    value.location = valueLocation;
    ::ad::map::lane::ContactTypeList valueTypes;
    ::ad::map::lane::ContactType valueTypesElement(::ad::map::lane::ContactType::INVALID);
    valueTypes.resize(1, valueTypesElement);
    value.types = valueTypes;
    ::ad::map::restriction::Restrictions valueRestrictions;
    ::ad::map::restriction::RestrictionList valueRestrictionsConjunctions;
    ::ad::map::restriction::Restriction valueRestrictionsConjunctionsElement;
    bool valueRestrictionsConjunctionsElementNegated{true};
    valueRestrictionsConjunctionsElement.negated = valueRestrictionsConjunctionsElementNegated;
    ::ad::map::restriction::RoadUserTypeList valueRestrictionsConjunctionsElementRoadUserTypes;
    ::ad::map::restriction::RoadUserType valueRestrictionsConjunctionsElementRoadUserTypesElement(
      ::ad::map::restriction::RoadUserType::INVALID);
    valueRestrictionsConjunctionsElementRoadUserTypes.resize(1,
                                                             valueRestrictionsConjunctionsElementRoadUserTypesElement);
    valueRestrictionsConjunctionsElement.roadUserTypes = valueRestrictionsConjunctionsElementRoadUserTypes;
    ::ad::map::restriction::PassengerCount valueRestrictionsConjunctionsElementPassengersMin(
      std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
    valueRestrictionsConjunctionsElement.passengersMin = valueRestrictionsConjunctionsElementPassengersMin;
    valueRestrictionsConjunctions.resize(1, valueRestrictionsConjunctionsElement);
    valueRestrictions.conjunctions = valueRestrictionsConjunctions;
    ::ad::map::restriction::RestrictionList valueRestrictionsDisjunctions;
    ::ad::map::restriction::Restriction valueRestrictionsDisjunctionsElement;
    bool valueRestrictionsDisjunctionsElementNegated{true};
    valueRestrictionsDisjunctionsElement.negated = valueRestrictionsDisjunctionsElementNegated;
    ::ad::map::restriction::RoadUserTypeList valueRestrictionsDisjunctionsElementRoadUserTypes;
    ::ad::map::restriction::RoadUserType valueRestrictionsDisjunctionsElementRoadUserTypesElement(
      ::ad::map::restriction::RoadUserType::INVALID);
    valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1,
                                                             valueRestrictionsDisjunctionsElementRoadUserTypesElement);
    valueRestrictionsDisjunctionsElement.roadUserTypes = valueRestrictionsDisjunctionsElementRoadUserTypes;
    ::ad::map::restriction::PassengerCount valueRestrictionsDisjunctionsElementPassengersMin(
      std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
    valueRestrictionsDisjunctionsElement.passengersMin = valueRestrictionsDisjunctionsElementPassengersMin;
    valueRestrictionsDisjunctions.resize(1, valueRestrictionsDisjunctionsElement);
    valueRestrictions.disjunctions = valueRestrictionsDisjunctions;
    value.restrictions = valueRestrictions;
    ::ad::map::landmark::LandmarkId valueTrafficLightId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
    value.trafficLightId = valueTrafficLightId;
    mValue = value;
  }

  ::ad::map::lane::ContactLane mValue;
};

TEST_F(ContactLaneTests, copyConstruction)
{
  ::ad::map::lane::ContactLane value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ContactLaneTests, moveConstruction)
{
  ::ad::map::lane::ContactLane tmpValue(mValue);
  ::ad::map::lane::ContactLane value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ContactLaneTests, copyAssignment)
{
  ::ad::map::lane::ContactLane value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ContactLaneTests, moveAssignment)
{
  ::ad::map::lane::ContactLane tmpValue(mValue);
  ::ad::map::lane::ContactLane value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ContactLaneTests, comparisonOperatorEqual)
{
  ::ad::map::lane::ContactLane valueA = mValue;
  ::ad::map::lane::ContactLane valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ContactLaneTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ContactLaneTests, comparisonOperatorToLaneDiffers)
{
  ::ad::map::lane::ContactLane valueA = mValue;
  ::ad::map::lane::LaneId toLane(std::numeric_limits<::ad::map::lane::LaneId>::max());
  valueA.toLane = toLane;
  ::ad::map::lane::ContactLane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ContactLaneTests, comparisonOperatorLocationDiffers)
{
  ::ad::map::lane::ContactLane valueA = mValue;
  ::ad::map::lane::ContactLocation location(::ad::map::lane::ContactLocation::OVERLAP);
  valueA.location = location;
  ::ad::map::lane::ContactLane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ContactLaneTests, comparisonOperatorTypesDiffers)
{
  ::ad::map::lane::ContactLane valueA = mValue;
  ::ad::map::lane::ContactTypeList types;
  ::ad::map::lane::ContactType typesElement(::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT);
  types.resize(2, typesElement);
  valueA.types = types;
  ::ad::map::lane::ContactLane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ContactLaneTests, comparisonOperatorRestrictionsDiffers)
{
  ::ad::map::lane::ContactLane valueA = mValue;
  ::ad::map::restriction::Restrictions restrictions;
  ::ad::map::restriction::RestrictionList restrictionsConjunctions;
  ::ad::map::restriction::Restriction restrictionsConjunctionsElement;
  bool restrictionsConjunctionsElementNegated{false};
  restrictionsConjunctionsElement.negated = restrictionsConjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList restrictionsConjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType restrictionsConjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::CAR_DIESEL);
  restrictionsConjunctionsElementRoadUserTypes.resize(2, restrictionsConjunctionsElementRoadUserTypesElement);
  restrictionsConjunctionsElement.roadUserTypes = restrictionsConjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount restrictionsConjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::max());
  restrictionsConjunctionsElement.passengersMin = restrictionsConjunctionsElementPassengersMin;
  restrictionsConjunctions.resize(2, restrictionsConjunctionsElement);
  restrictions.conjunctions = restrictionsConjunctions;
  ::ad::map::restriction::RestrictionList restrictionsDisjunctions;
  ::ad::map::restriction::Restriction restrictionsDisjunctionsElement;
  bool restrictionsDisjunctionsElementNegated{false};
  restrictionsDisjunctionsElement.negated = restrictionsDisjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList restrictionsDisjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType restrictionsDisjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::CAR_DIESEL);
  restrictionsDisjunctionsElementRoadUserTypes.resize(2, restrictionsDisjunctionsElementRoadUserTypesElement);
  restrictionsDisjunctionsElement.roadUserTypes = restrictionsDisjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount restrictionsDisjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::max());
  restrictionsDisjunctionsElement.passengersMin = restrictionsDisjunctionsElementPassengersMin;
  restrictionsDisjunctions.resize(2, restrictionsDisjunctionsElement);
  restrictions.disjunctions = restrictionsDisjunctions;
  valueA.restrictions = restrictions;
  ::ad::map::lane::ContactLane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ContactLaneTests, comparisonOperatorTrafficLightIdDiffers)
{
  ::ad::map::lane::ContactLane valueA = mValue;
  ::ad::map::landmark::LandmarkId trafficLightId(std::numeric_limits<::ad::map::landmark::LandmarkId>::max());
  valueA.trafficLightId = trafficLightId;
  ::ad::map::lane::ContactLane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
