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
#include "ad/map/restriction/Restrictions.hpp"

class RestrictionsTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::restriction::Restrictions value;
    ::ad::map::restriction::RestrictionList valueConjunctions;
    ::ad::map::restriction::Restriction valueConjunctionsElement;
    bool valueConjunctionsElementNegated{true};
    valueConjunctionsElement.negated = valueConjunctionsElementNegated;
    ::ad::map::restriction::RoadUserTypeList valueConjunctionsElementRoadUserTypes;
    ::ad::map::restriction::RoadUserType valueConjunctionsElementRoadUserTypesElement(
      ::ad::map::restriction::RoadUserType::INVALID);
    valueConjunctionsElementRoadUserTypes.resize(1, valueConjunctionsElementRoadUserTypesElement);
    valueConjunctionsElement.roadUserTypes = valueConjunctionsElementRoadUserTypes;
    ::ad::map::restriction::PassengerCount valueConjunctionsElementPassengersMin(
      std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
    valueConjunctionsElement.passengersMin = valueConjunctionsElementPassengersMin;
    valueConjunctions.resize(1, valueConjunctionsElement);
    value.conjunctions = valueConjunctions;
    ::ad::map::restriction::RestrictionList valueDisjunctions;
    ::ad::map::restriction::Restriction valueDisjunctionsElement;
    bool valueDisjunctionsElementNegated{true};
    valueDisjunctionsElement.negated = valueDisjunctionsElementNegated;
    ::ad::map::restriction::RoadUserTypeList valueDisjunctionsElementRoadUserTypes;
    ::ad::map::restriction::RoadUserType valueDisjunctionsElementRoadUserTypesElement(
      ::ad::map::restriction::RoadUserType::INVALID);
    valueDisjunctionsElementRoadUserTypes.resize(1, valueDisjunctionsElementRoadUserTypesElement);
    valueDisjunctionsElement.roadUserTypes = valueDisjunctionsElementRoadUserTypes;
    ::ad::map::restriction::PassengerCount valueDisjunctionsElementPassengersMin(
      std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
    valueDisjunctionsElement.passengersMin = valueDisjunctionsElementPassengersMin;
    valueDisjunctions.resize(1, valueDisjunctionsElement);
    value.disjunctions = valueDisjunctions;
    mValue = value;
  }

  ::ad::map::restriction::Restrictions mValue;
};

TEST_F(RestrictionsTests, copyConstruction)
{
  ::ad::map::restriction::Restrictions value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(RestrictionsTests, moveConstruction)
{
  ::ad::map::restriction::Restrictions tmpValue(mValue);
  ::ad::map::restriction::Restrictions value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(RestrictionsTests, copyAssignment)
{
  ::ad::map::restriction::Restrictions value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(RestrictionsTests, moveAssignment)
{
  ::ad::map::restriction::Restrictions tmpValue(mValue);
  ::ad::map::restriction::Restrictions value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(RestrictionsTests, comparisonOperatorEqual)
{
  ::ad::map::restriction::Restrictions valueA = mValue;
  ::ad::map::restriction::Restrictions valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(RestrictionsTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(RestrictionsTests, comparisonOperatorConjunctionsDiffers)
{
  ::ad::map::restriction::Restrictions valueA = mValue;
  ::ad::map::restriction::RestrictionList conjunctions;
  ::ad::map::restriction::Restriction conjunctionsElement;
  bool conjunctionsElementNegated{false};
  conjunctionsElement.negated = conjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList conjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType conjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::CAR_DIESEL);
  conjunctionsElementRoadUserTypes.resize(2, conjunctionsElementRoadUserTypesElement);
  conjunctionsElement.roadUserTypes = conjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount conjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::max());
  conjunctionsElement.passengersMin = conjunctionsElementPassengersMin;
  conjunctions.resize(2, conjunctionsElement);
  valueA.conjunctions = conjunctions;
  ::ad::map::restriction::Restrictions valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(RestrictionsTests, comparisonOperatorDisjunctionsDiffers)
{
  ::ad::map::restriction::Restrictions valueA = mValue;
  ::ad::map::restriction::RestrictionList disjunctions;
  ::ad::map::restriction::Restriction disjunctionsElement;
  bool disjunctionsElementNegated{false};
  disjunctionsElement.negated = disjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList disjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType disjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::CAR_DIESEL);
  disjunctionsElementRoadUserTypes.resize(2, disjunctionsElementRoadUserTypesElement);
  disjunctionsElement.roadUserTypes = disjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount disjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::max());
  disjunctionsElement.passengersMin = disjunctionsElementPassengersMin;
  disjunctions.resize(2, disjunctionsElement);
  valueA.disjunctions = disjunctions;
  ::ad::map::restriction::Restrictions valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
