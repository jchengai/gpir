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
#include "ad/map/point/Longitude.hpp"

TEST(LongitudeTests, defaultConstructionIsInvalid)
{
  ::ad::map::point::Longitude value;
  EXPECT_FALSE(value.isValid());
}

TEST(LongitudeTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::map::point::Longitude::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-8, ::ad::map::point::Longitude::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::map::point::Longitude::cPrecisionValue,
                   static_cast<double>(::ad::map::point::Longitude::getPrecision()));
}

TEST(LongitudeTests, minIsValid)
{
  EXPECT_TRUE(::ad::map::point::Longitude::getMin().isValid());
}

TEST(LongitudeTests, aboveMinIsValid)
{
  ::ad::map::point::Longitude value(::ad::map::point::Longitude::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(LongitudeTests, belowMinIsInvalid)
{
  ::ad::map::point::Longitude value(::ad::map::point::Longitude::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(LongitudeTests, maxIsValid)
{
  EXPECT_TRUE(::ad::map::point::Longitude::getMax().isValid());
}

TEST(LongitudeTests, aboveMaxIsInvalid)
{
  ::ad::map::point::Longitude value(::ad::map::point::Longitude::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(LongitudeTests, belowMaxIsValid)
{
  ::ad::map::point::Longitude value(::ad::map::point::Longitude::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_MAP_POINT_LONGITUDE_THROWS_EXCEPTION == 1)
TEST(LongitudeTests, ensureValidThrowsOnInvalid)
{
  ::ad::map::point::Longitude value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(LongitudeTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::map::point::Longitude value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(LongitudeTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::map::point::Longitude value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(LongitudeTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::Longitude::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::Longitude>::lowest()));
}

TEST(LongitudeTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::Longitude::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::Longitude>::max()));
}

TEST(LongitudeTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::Longitude::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::Longitude>::epsilon()));
}

TEST(LongitudeTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::map::point::Longitude(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::map::point::Longitude(-1.))));
  EXPECT_DOUBLE_EQ(
    ::ad::map::point::Longitude::cPrecisionValue,
    static_cast<double>(std::fabs(::ad::map::point::Longitude(::ad::map::point::Longitude::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::map::point::Longitude::cMinValue),
                   static_cast<double>(std::fabs(::ad::map::point::Longitude(::ad::map::point::Longitude::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::Longitude::cMinValue),
    static_cast<double>(std::fabs(::ad::map::point::Longitude(-::ad::map::point::Longitude::cMinValue))));
  EXPECT_DOUBLE_EQ(std::fabs(::ad::map::point::Longitude::cMaxValue),
                   static_cast<double>(std::fabs(::ad::map::point::Longitude(::ad::map::point::Longitude::cMaxValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::Longitude::cMaxValue),
    static_cast<double>(std::fabs(::ad::map::point::Longitude(-::ad::map::point::Longitude::cMaxValue))));
}

TEST(LongitudeTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::map::point::Longitude::cMinValue;
  ::ad::map::point::Longitude value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(LongitudeTests, copyConstructionFromValidValue)
{
  ::ad::map::point::Longitude const validValue(::ad::map::point::Longitude::cMinValue);
  ::ad::map::point::Longitude value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(LongitudeTests, moveConstructionFromValidValue)
{
  ::ad::map::point::Longitude validValue(::ad::map::point::Longitude::cMinValue);
  ::ad::map::point::Longitude value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::Longitude::cMinValue, static_cast<double>(value));
}

TEST(LongitudeTests, assignmentFromValidValue)
{
  ::ad::map::point::Longitude const validValue(::ad::map::point::Longitude::cMinValue);
  ::ad::map::point::Longitude value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(LongitudeTests, moveAssignmentFromValidValue)
{
  ::ad::map::point::Longitude validValue(::ad::map::point::Longitude::cMinValue);
  ::ad::map::point::Longitude value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::Longitude::cMinValue, static_cast<double>(value));
}

TEST(LongitudeTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::map::point::Longitude value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(LongitudeTests, copyConstructionFromInvalidValue)
{
  ::ad::map::point::Longitude const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::Longitude value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(LongitudeTests, assignmentFromInvalidValue)
{
  ::ad::map::point::Longitude const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::Longitude value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(LongitudeTests, selfAssignment)
{
  ::ad::map::point::Longitude value(::ad::map::point::Longitude::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(LongitudeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::point::Longitude value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_MAP_POINT_LONGITUDE_THROWS_EXCEPTION == 1)
TEST(LongitudeTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::map::point::Longitude const value(::ad::map::point::Longitude::cMinValue);
  ::ad::map::point::Longitude const invalidValue;

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

TEST(LongitudeTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::map::point::Longitude const minimalValue(::ad::map::point::Longitude::cMinValue);
  ::ad::map::point::Longitude const maximalValue(::ad::map::point::Longitude::cMaxValue);
  ::ad::map::point::Longitude const invalidValue;
  ::ad::map::point::Longitude calculationValue;

  //  operator+(::ad::map::point::Longitude)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::map::point::Longitude)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::map::point::Longitude)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::map::point::Longitude)
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

  //  operator/(::ad::map::point::Longitude)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::map::point::Longitude(0.0), std::out_of_range);

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

TEST(LongitudeTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::map::point::Longitude::cPrecisionValue * 10.;
  ::ad::map::point::Longitude value;
  if (::ad::map::point::Longitude::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::Longitude(::ad::map::point::Longitude::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::Longitude::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::Longitude(::ad::map::point::Longitude::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::Longitude(precisionValueTimesTen);
  }
  ::ad::map::point::Longitude const sameValue = value;
  ::ad::map::point::Longitude const slightlyBiggerValue(static_cast<double>(value)
                                                        + ::ad::map::point::Longitude::cPrecisionValue * 0.9);
  ::ad::map::point::Longitude const slightlySmallerValue(static_cast<double>(value)
                                                         - ::ad::map::point::Longitude::cPrecisionValue * 0.9);
  ::ad::map::point::Longitude const actuallyBiggerValue(static_cast<double>(value)
                                                        + ::ad::map::point::Longitude::cPrecisionValue * 1.1);
  ::ad::map::point::Longitude const actuallySmallerValue(static_cast<double>(value)
                                                         - ::ad::map::point::Longitude::cPrecisionValue * 1.1);

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

TEST(LongitudeTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::map::point::Longitude::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::map::point::Longitude::cPrecisionValue * 10.;
  ::ad::map::point::Longitude value;
  if (::ad::map::point::Longitude::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::Longitude(::ad::map::point::Longitude::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::Longitude::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::Longitude(::ad::map::point::Longitude::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::Longitude(precisionValueTimesTen);
  }

  ::ad::map::point::Longitude result;

  //  operator+(::ad::map::point::Longitude)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::map::point::Longitude)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::map::point::Longitude)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::map::point::Longitude)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator*(double, ::ad::map::point::Longitude)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::map::point::Longitude)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::map::point::Longitude::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::map::point::Longitude::cMaxValue))
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
