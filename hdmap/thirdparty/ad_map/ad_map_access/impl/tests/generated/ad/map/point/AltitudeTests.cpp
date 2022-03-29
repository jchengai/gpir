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
#include "ad/map/point/Altitude.hpp"

TEST(AltitudeTests, defaultConstructionIsInvalid)
{
  ::ad::map::point::Altitude value;
  EXPECT_FALSE(value.isValid());
}

TEST(AltitudeTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::map::point::Altitude::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-3, ::ad::map::point::Altitude::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::map::point::Altitude::cPrecisionValue,
                   static_cast<double>(::ad::map::point::Altitude::getPrecision()));
}

TEST(AltitudeTests, minIsValid)
{
  EXPECT_TRUE(::ad::map::point::Altitude::getMin().isValid());
}

TEST(AltitudeTests, aboveMinIsValid)
{
  ::ad::map::point::Altitude value(::ad::map::point::Altitude::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(AltitudeTests, belowMinIsInvalid)
{
  ::ad::map::point::Altitude value(::ad::map::point::Altitude::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(AltitudeTests, maxIsValid)
{
  EXPECT_TRUE(::ad::map::point::Altitude::getMax().isValid());
}

TEST(AltitudeTests, aboveMaxIsInvalid)
{
  ::ad::map::point::Altitude value(::ad::map::point::Altitude::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(AltitudeTests, belowMaxIsValid)
{
  ::ad::map::point::Altitude value(::ad::map::point::Altitude::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_MAP_POINT_ALTITUDE_THROWS_EXCEPTION == 1)
TEST(AltitudeTests, ensureValidThrowsOnInvalid)
{
  ::ad::map::point::Altitude value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(AltitudeTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::map::point::Altitude value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(AltitudeTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::map::point::Altitude value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(AltitudeTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::Altitude::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::Altitude>::lowest()));
}

TEST(AltitudeTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::Altitude::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::Altitude>::max()));
}

TEST(AltitudeTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::Altitude::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::Altitude>::epsilon()));
}

TEST(AltitudeTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::map::point::Altitude(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::map::point::Altitude(-1.))));
  EXPECT_DOUBLE_EQ(
    ::ad::map::point::Altitude::cPrecisionValue,
    static_cast<double>(std::fabs(::ad::map::point::Altitude(::ad::map::point::Altitude::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::map::point::Altitude::cMinValue),
                   static_cast<double>(std::fabs(::ad::map::point::Altitude(::ad::map::point::Altitude::cMinValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::map::point::Altitude::cMinValue),
                   static_cast<double>(std::fabs(::ad::map::point::Altitude(-::ad::map::point::Altitude::cMinValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::map::point::Altitude::cMaxValue),
                   static_cast<double>(std::fabs(::ad::map::point::Altitude(::ad::map::point::Altitude::cMaxValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::map::point::Altitude::cMaxValue),
                   static_cast<double>(std::fabs(::ad::map::point::Altitude(-::ad::map::point::Altitude::cMaxValue))));
}

TEST(AltitudeTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::map::point::Altitude::cMinValue;
  ::ad::map::point::Altitude value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(AltitudeTests, copyConstructionFromValidValue)
{
  ::ad::map::point::Altitude const validValue(::ad::map::point::Altitude::cMinValue);
  ::ad::map::point::Altitude value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(AltitudeTests, moveConstructionFromValidValue)
{
  ::ad::map::point::Altitude validValue(::ad::map::point::Altitude::cMinValue);
  ::ad::map::point::Altitude value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::Altitude::cMinValue, static_cast<double>(value));
}

TEST(AltitudeTests, assignmentFromValidValue)
{
  ::ad::map::point::Altitude const validValue(::ad::map::point::Altitude::cMinValue);
  ::ad::map::point::Altitude value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(AltitudeTests, moveAssignmentFromValidValue)
{
  ::ad::map::point::Altitude validValue(::ad::map::point::Altitude::cMinValue);
  ::ad::map::point::Altitude value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::Altitude::cMinValue, static_cast<double>(value));
}

TEST(AltitudeTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::map::point::Altitude value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(AltitudeTests, copyConstructionFromInvalidValue)
{
  ::ad::map::point::Altitude const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::Altitude value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(AltitudeTests, assignmentFromInvalidValue)
{
  ::ad::map::point::Altitude const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::Altitude value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(AltitudeTests, selfAssignment)
{
  ::ad::map::point::Altitude value(::ad::map::point::Altitude::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(AltitudeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::point::Altitude value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_MAP_POINT_ALTITUDE_THROWS_EXCEPTION == 1)
TEST(AltitudeTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::map::point::Altitude const value(::ad::map::point::Altitude::cMinValue);
  ::ad::map::point::Altitude const invalidValue;

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

TEST(AltitudeTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::map::point::Altitude const minimalValue(::ad::map::point::Altitude::cMinValue);
  ::ad::map::point::Altitude const maximalValue(::ad::map::point::Altitude::cMaxValue);
  ::ad::map::point::Altitude const invalidValue;
  ::ad::map::point::Altitude calculationValue;

  //  operator+(::ad::map::point::Altitude)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::map::point::Altitude)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::map::point::Altitude)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::map::point::Altitude)
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

  //  operator/(::ad::map::point::Altitude)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::map::point::Altitude(0.0), std::out_of_range);

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

TEST(AltitudeTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::map::point::Altitude::cPrecisionValue * 10.;
  ::ad::map::point::Altitude value;
  if (::ad::map::point::Altitude::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::Altitude(::ad::map::point::Altitude::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::Altitude::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::Altitude(::ad::map::point::Altitude::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::Altitude(precisionValueTimesTen);
  }
  ::ad::map::point::Altitude const sameValue = value;
  ::ad::map::point::Altitude const slightlyBiggerValue(static_cast<double>(value)
                                                       + ::ad::map::point::Altitude::cPrecisionValue * 0.9);
  ::ad::map::point::Altitude const slightlySmallerValue(static_cast<double>(value)
                                                        - ::ad::map::point::Altitude::cPrecisionValue * 0.9);
  ::ad::map::point::Altitude const actuallyBiggerValue(static_cast<double>(value)
                                                       + ::ad::map::point::Altitude::cPrecisionValue * 1.1);
  ::ad::map::point::Altitude const actuallySmallerValue(static_cast<double>(value)
                                                        - ::ad::map::point::Altitude::cPrecisionValue * 1.1);

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

TEST(AltitudeTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::map::point::Altitude::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::map::point::Altitude::cPrecisionValue * 10.;
  ::ad::map::point::Altitude value;
  if (::ad::map::point::Altitude::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::Altitude(::ad::map::point::Altitude::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::Altitude::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::Altitude(::ad::map::point::Altitude::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::Altitude(precisionValueTimesTen);
  }

  ::ad::map::point::Altitude result;

  //  operator+(::ad::map::point::Altitude)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::map::point::Altitude)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::map::point::Altitude)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::map::point::Altitude)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator*(double, ::ad::map::point::Altitude)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::map::point::Altitude)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::map::point::Altitude::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::map::point::Altitude::cMaxValue))
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
