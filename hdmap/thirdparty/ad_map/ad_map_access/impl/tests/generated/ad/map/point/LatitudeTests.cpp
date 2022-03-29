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
#include "ad/map/point/Latitude.hpp"

TEST(LatitudeTests, defaultConstructionIsInvalid)
{
  ::ad::map::point::Latitude value;
  EXPECT_FALSE(value.isValid());
}

TEST(LatitudeTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::map::point::Latitude::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-8, ::ad::map::point::Latitude::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::map::point::Latitude::cPrecisionValue,
                   static_cast<double>(::ad::map::point::Latitude::getPrecision()));
}

TEST(LatitudeTests, minIsValid)
{
  EXPECT_TRUE(::ad::map::point::Latitude::getMin().isValid());
}

TEST(LatitudeTests, aboveMinIsValid)
{
  ::ad::map::point::Latitude value(::ad::map::point::Latitude::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(LatitudeTests, belowMinIsInvalid)
{
  ::ad::map::point::Latitude value(::ad::map::point::Latitude::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(LatitudeTests, maxIsValid)
{
  EXPECT_TRUE(::ad::map::point::Latitude::getMax().isValid());
}

TEST(LatitudeTests, aboveMaxIsInvalid)
{
  ::ad::map::point::Latitude value(::ad::map::point::Latitude::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(LatitudeTests, belowMaxIsValid)
{
  ::ad::map::point::Latitude value(::ad::map::point::Latitude::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_MAP_POINT_LATITUDE_THROWS_EXCEPTION == 1)
TEST(LatitudeTests, ensureValidThrowsOnInvalid)
{
  ::ad::map::point::Latitude value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(LatitudeTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::map::point::Latitude value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(LatitudeTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::map::point::Latitude value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(LatitudeTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::Latitude::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::Latitude>::lowest()));
}

TEST(LatitudeTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::Latitude::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::Latitude>::max()));
}

TEST(LatitudeTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::Latitude::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::Latitude>::epsilon()));
}

TEST(LatitudeTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::map::point::Latitude(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::map::point::Latitude(-1.))));
  EXPECT_DOUBLE_EQ(
    ::ad::map::point::Latitude::cPrecisionValue,
    static_cast<double>(std::fabs(::ad::map::point::Latitude(::ad::map::point::Latitude::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::map::point::Latitude::cMinValue),
                   static_cast<double>(std::fabs(::ad::map::point::Latitude(::ad::map::point::Latitude::cMinValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::map::point::Latitude::cMinValue),
                   static_cast<double>(std::fabs(::ad::map::point::Latitude(-::ad::map::point::Latitude::cMinValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::map::point::Latitude::cMaxValue),
                   static_cast<double>(std::fabs(::ad::map::point::Latitude(::ad::map::point::Latitude::cMaxValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::map::point::Latitude::cMaxValue),
                   static_cast<double>(std::fabs(::ad::map::point::Latitude(-::ad::map::point::Latitude::cMaxValue))));
}

TEST(LatitudeTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::map::point::Latitude::cMinValue;
  ::ad::map::point::Latitude value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(LatitudeTests, copyConstructionFromValidValue)
{
  ::ad::map::point::Latitude const validValue(::ad::map::point::Latitude::cMinValue);
  ::ad::map::point::Latitude value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(LatitudeTests, moveConstructionFromValidValue)
{
  ::ad::map::point::Latitude validValue(::ad::map::point::Latitude::cMinValue);
  ::ad::map::point::Latitude value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::Latitude::cMinValue, static_cast<double>(value));
}

TEST(LatitudeTests, assignmentFromValidValue)
{
  ::ad::map::point::Latitude const validValue(::ad::map::point::Latitude::cMinValue);
  ::ad::map::point::Latitude value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(LatitudeTests, moveAssignmentFromValidValue)
{
  ::ad::map::point::Latitude validValue(::ad::map::point::Latitude::cMinValue);
  ::ad::map::point::Latitude value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::Latitude::cMinValue, static_cast<double>(value));
}

TEST(LatitudeTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::map::point::Latitude value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(LatitudeTests, copyConstructionFromInvalidValue)
{
  ::ad::map::point::Latitude const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::Latitude value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(LatitudeTests, assignmentFromInvalidValue)
{
  ::ad::map::point::Latitude const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::Latitude value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(LatitudeTests, selfAssignment)
{
  ::ad::map::point::Latitude value(::ad::map::point::Latitude::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(LatitudeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::point::Latitude value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_MAP_POINT_LATITUDE_THROWS_EXCEPTION == 1)
TEST(LatitudeTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::map::point::Latitude const value(::ad::map::point::Latitude::cMinValue);
  ::ad::map::point::Latitude const invalidValue;

  EXPECT_THROW((void)(invalidValue == value), std::out_of_range);
  EXPECT_THROW((void)(value == invalidValue), std::out_of_range);

  EXPECT_THROW((void)(invalidValue != value), std::out_of_range);
  EXPECT_THROW((void)(value != invalidValue), std::out_of_range);

  EXPECT_THROW((void)(invalidValue > value), std::out_of_range);
  EXPECT_THROW((void)(value > invalidValue), std::out_of_range);

  EXPECT_THROW((void)(invalidValue < value), std::out_of_range);
  EXPECT_THROW((void)(value < invalidValue), std::out_of_range);

  EXPECT_THROW((void)(invalidValue >= value), std::out_of_range);
  EXPECT_THROW((void)(value >= invalidValue), std::out_of_range);

  EXPECT_THROW((void)(invalidValue <= value), std::out_of_range);
  EXPECT_THROW((void)(value <= invalidValue), std::out_of_range);
}

TEST(LatitudeTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::map::point::Latitude const minimalValue(::ad::map::point::Latitude::cMinValue);
  ::ad::map::point::Latitude const maximalValue(::ad::map::point::Latitude::cMaxValue);
  ::ad::map::point::Latitude const invalidValue;
  ::ad::map::point::Latitude calculationValue;

  //  operator+(::ad::map::point::Latitude)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::map::point::Latitude)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::map::point::Latitude)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::map::point::Latitude)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue -= minimalValue, std::out_of_range);
  calculationValue = minimalValue;
  EXPECT_THROW(calculationValue -= invalidValue, std::out_of_range);
  calculationValue = minimalValue;
  EXPECT_THROW(calculationValue -= maximalValue, std::out_of_range);

  //  operator*(double)
  EXPECT_THROW(invalidValue * static_cast<double>(maximalValue), std::out_of_range);
  EXPECT_THROW(maximalValue * static_cast<double>(maximalValue), std::out_of_range);

  //  operator/(double)
  EXPECT_THROW(invalidValue / static_cast<double>(maximalValue), std::out_of_range);
  EXPECT_THROW(maximalValue / static_cast<double>(invalidValue), std::out_of_range);
  EXPECT_THROW(maximalValue / 0.0, std::out_of_range);
  EXPECT_THROW(maximalValue / 0.5, std::out_of_range);

  //  operator/(::ad::map::point::Latitude)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::map::point::Latitude(0.0), std::out_of_range);

  //  operator-()
  EXPECT_THROW(-invalidValue, std::out_of_range);
  if (std::fabs(static_cast<double>(maximalValue)) > std::fabs(static_cast<double>(minimalValue)))
  {
    EXPECT_THROW(-maximalValue, std::out_of_range);
  }
  else if (std::fabs(static_cast<double>(maximalValue)) < std::fabs(static_cast<double>(minimalValue)))
  {
    EXPECT_THROW(-minimalValue, std::out_of_range);
  }
  else
  {
    // impossible to trigger invalid result while operand is valid
  }
}
#endif

TEST(LatitudeTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::map::point::Latitude::cPrecisionValue * 10.;
  ::ad::map::point::Latitude value;
  if (::ad::map::point::Latitude::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::Latitude(::ad::map::point::Latitude::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::Latitude::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::Latitude(::ad::map::point::Latitude::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::Latitude(precisionValueTimesTen);
  }
  ::ad::map::point::Latitude const sameValue = value;
  ::ad::map::point::Latitude const slightlyBiggerValue(static_cast<double>(value)
                                                       + ::ad::map::point::Latitude::cPrecisionValue * 0.9);
  ::ad::map::point::Latitude const slightlySmallerValue(static_cast<double>(value)
                                                        - ::ad::map::point::Latitude::cPrecisionValue * 0.9);
  ::ad::map::point::Latitude const actuallyBiggerValue(static_cast<double>(value)
                                                       + ::ad::map::point::Latitude::cPrecisionValue * 1.1);
  ::ad::map::point::Latitude const actuallySmallerValue(static_cast<double>(value)
                                                        - ::ad::map::point::Latitude::cPrecisionValue * 1.1);

  // operator ==
  EXPECT_TRUE(value == sameValue);
  EXPECT_TRUE(value == slightlyBiggerValue);
  EXPECT_TRUE(value == slightlySmallerValue);
  EXPECT_FALSE(value == actuallyBiggerValue);
  EXPECT_FALSE(value == actuallySmallerValue);

  // operator !=
  EXPECT_FALSE(value != sameValue);
  EXPECT_FALSE(value != slightlyBiggerValue);
  EXPECT_FALSE(value != slightlySmallerValue);
  EXPECT_TRUE(value != actuallyBiggerValue);
  EXPECT_TRUE(value != actuallySmallerValue);

  // operator >
  EXPECT_FALSE(value > value);
  EXPECT_FALSE(slightlyBiggerValue > value);
  EXPECT_TRUE(actuallyBiggerValue > value);

  // operator >=
  EXPECT_FALSE(actuallySmallerValue >= value);
  EXPECT_TRUE(slightlySmallerValue >= value);
  EXPECT_TRUE(value >= value);
  EXPECT_TRUE(slightlyBiggerValue >= value);
  EXPECT_TRUE(actuallyBiggerValue >= value);

  // operator <
  EXPECT_FALSE(value < value);
  EXPECT_FALSE(slightlySmallerValue < value);
  EXPECT_TRUE(actuallySmallerValue < value);

  // operator <=
  EXPECT_FALSE(actuallyBiggerValue <= value);
  EXPECT_TRUE(slightlyBiggerValue <= value);
  EXPECT_TRUE(value <= value);
  EXPECT_TRUE(slightlySmallerValue <= value);
  EXPECT_TRUE(actuallySmallerValue <= value);
}

TEST(LatitudeTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::map::point::Latitude::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::map::point::Latitude::cPrecisionValue * 10.;
  ::ad::map::point::Latitude value;
  if (::ad::map::point::Latitude::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::Latitude(::ad::map::point::Latitude::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::Latitude::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::Latitude(::ad::map::point::Latitude::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::Latitude(precisionValueTimesTen);
  }

  ::ad::map::point::Latitude result;

  //  operator+(::ad::map::point::Latitude)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::map::point::Latitude)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::map::point::Latitude)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::map::point::Latitude)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator*(double, ::ad::map::point::Latitude)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::map::point::Latitude)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::map::point::Latitude::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::map::point::Latitude::cMaxValue))
  {
    result = -value;
  }
  else
  {
    // not clear on how to trigger valid result if such small value is not working
  }
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
