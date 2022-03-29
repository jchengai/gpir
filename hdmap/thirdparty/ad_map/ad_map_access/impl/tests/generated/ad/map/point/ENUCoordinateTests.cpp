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
#include "ad/map/point/ENUCoordinate.hpp"

TEST(ENUCoordinateTests, defaultConstructionIsInvalid)
{
  ::ad::map::point::ENUCoordinate value;
  EXPECT_FALSE(value.isValid());
}

TEST(ENUCoordinateTests, minIsDefinedAsExpected)
{
  EXPECT_DOUBLE_EQ(-1e6, ::ad::map::point::ENUCoordinate::cMinValue);
  EXPECT_DOUBLE_EQ(::ad::map::point::ENUCoordinate::cMinValue,
                   static_cast<double>(::ad::map::point::ENUCoordinate::getMin()));
}

TEST(ENUCoordinateTests, maxIsDefinedAsExpected)
{
  EXPECT_DOUBLE_EQ(1e6, ::ad::map::point::ENUCoordinate::cMaxValue);
  EXPECT_DOUBLE_EQ(::ad::map::point::ENUCoordinate::cMaxValue,
                   static_cast<double>(::ad::map::point::ENUCoordinate::getMax()));
}

TEST(ENUCoordinateTests, precisionIsDefinedAsExpected)
{
  EXPECT_LT(0., ::ad::map::point::ENUCoordinate::cPrecisionValue);
  EXPECT_DOUBLE_EQ(1e-3, ::ad::map::point::ENUCoordinate::cPrecisionValue);
  EXPECT_DOUBLE_EQ(::ad::map::point::ENUCoordinate::cPrecisionValue,
                   static_cast<double>(::ad::map::point::ENUCoordinate::getPrecision()));
}

TEST(ENUCoordinateTests, minIsValid)
{
  EXPECT_TRUE(::ad::map::point::ENUCoordinate::getMin().isValid());
}

TEST(ENUCoordinateTests, aboveMinIsValid)
{
  ::ad::map::point::ENUCoordinate value(::ad::map::point::ENUCoordinate::cMinValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

TEST(ENUCoordinateTests, belowMinIsInvalid)
{
  ::ad::map::point::ENUCoordinate value(::ad::map::point::ENUCoordinate::cMinValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(ENUCoordinateTests, maxIsValid)
{
  EXPECT_TRUE(::ad::map::point::ENUCoordinate::getMax().isValid());
}

TEST(ENUCoordinateTests, aboveMaxIsInvalid)
{
  ::ad::map::point::ENUCoordinate value(::ad::map::point::ENUCoordinate::cMaxValue * 1.1);
  EXPECT_FALSE(value.isValid());
}

TEST(ENUCoordinateTests, belowMaxIsValid)
{
  ::ad::map::point::ENUCoordinate value(::ad::map::point::ENUCoordinate::cMaxValue * 0.9);
  EXPECT_TRUE(value.isValid());
}

#if (AD_MAP_POINT_ENUCOORDINATE_THROWS_EXCEPTION == 1)
TEST(ENUCoordinateTests, ensureValidThrowsOnInvalid)
{
  ::ad::map::point::ENUCoordinate value;
  EXPECT_THROW(value.ensureValid(), std::out_of_range);
}

TEST(ENUCoordinateTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::map::point::ENUCoordinate value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(ENUCoordinateTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::map::point::ENUCoordinate value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(ENUCoordinateTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::ENUCoordinate::getMin()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::ENUCoordinate>::lowest()));
}

TEST(ENUCoordinateTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::ENUCoordinate::getMax()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::ENUCoordinate>::max()));
}

TEST(ENUCoordinateTestsStd, numericLimitsEpsilonIsPrecision)
{
  EXPECT_DOUBLE_EQ(static_cast<double>(::ad::map::point::ENUCoordinate::getPrecision()),
                   static_cast<double>(std::numeric_limits<::ad::map::point::ENUCoordinate>::epsilon()));
}

TEST(ENUCoordinateTestsStd, fabsIsWorkingCorrectly)
{
  EXPECT_DOUBLE_EQ(0., static_cast<double>(std::fabs(::ad::map::point::ENUCoordinate(-0.))));
  EXPECT_DOUBLE_EQ(1., static_cast<double>(std::fabs(::ad::map::point::ENUCoordinate(-1.))));
  EXPECT_DOUBLE_EQ(
    ::ad::map::point::ENUCoordinate::cPrecisionValue,
    static_cast<double>(std::fabs(::ad::map::point::ENUCoordinate(::ad::map::point::ENUCoordinate::cPrecisionValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ENUCoordinate::cMinValue),
    static_cast<double>(std::fabs(::ad::map::point::ENUCoordinate(::ad::map::point::ENUCoordinate::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ENUCoordinate::cMinValue),
    static_cast<double>(std::fabs(::ad::map::point::ENUCoordinate(-::ad::map::point::ENUCoordinate::cMinValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ENUCoordinate::cMaxValue),
    static_cast<double>(std::fabs(::ad::map::point::ENUCoordinate(::ad::map::point::ENUCoordinate::cMaxValue))));
  EXPECT_DOUBLE_EQ(
    std::fabs(::ad::map::point::ENUCoordinate::cMaxValue),
    static_cast<double>(std::fabs(::ad::map::point::ENUCoordinate(-::ad::map::point::ENUCoordinate::cMaxValue))));
}

TEST(ENUCoordinateTests, constructionFromValidFPValue)
{
  double const validValue = ::ad::map::point::ENUCoordinate::cMinValue;
  ::ad::map::point::ENUCoordinate value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(validValue, static_cast<double>(value));
}

TEST(ENUCoordinateTests, copyConstructionFromValidValue)
{
  ::ad::map::point::ENUCoordinate const validValue(::ad::map::point::ENUCoordinate::cMinValue);
  ::ad::map::point::ENUCoordinate value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(ENUCoordinateTests, moveConstructionFromValidValue)
{
  ::ad::map::point::ENUCoordinate validValue(::ad::map::point::ENUCoordinate::cMinValue);
  ::ad::map::point::ENUCoordinate value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::ENUCoordinate::cMinValue, static_cast<double>(value));
}

TEST(ENUCoordinateTests, assignmentFromValidValue)
{
  ::ad::map::point::ENUCoordinate const validValue(::ad::map::point::ENUCoordinate::cMinValue);
  ::ad::map::point::ENUCoordinate value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(static_cast<double>(validValue), static_cast<double>(value));
}

TEST(ENUCoordinateTests, moveAssignmentFromValidValue)
{
  ::ad::map::point::ENUCoordinate validValue(::ad::map::point::ENUCoordinate::cMinValue);
  ::ad::map::point::ENUCoordinate value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_DOUBLE_EQ(::ad::map::point::ENUCoordinate::cMinValue, static_cast<double>(value));
}

TEST(ENUCoordinateTests, constructionFromInvalidFPValue)
{
  double const invalidValue = std::numeric_limits<double>::quiet_NaN();
  ::ad::map::point::ENUCoordinate value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(ENUCoordinateTests, copyConstructionFromInvalidValue)
{
  ::ad::map::point::ENUCoordinate const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::ENUCoordinate value(invalidValue);
  EXPECT_FALSE(value.isValid());
}

TEST(ENUCoordinateTests, assignmentFromInvalidValue)
{
  ::ad::map::point::ENUCoordinate const invalidValue(std::numeric_limits<double>::quiet_NaN());
  ::ad::map::point::ENUCoordinate value;
  value = invalidValue;
  EXPECT_FALSE(value.isValid());
}

TEST(ENUCoordinateTests, selfAssignment)
{
  ::ad::map::point::ENUCoordinate value(::ad::map::point::ENUCoordinate::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(ENUCoordinateTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::point::ENUCoordinate value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if (AD_MAP_POINT_ENUCOORDINATE_THROWS_EXCEPTION == 1)
TEST(ENUCoordinateTests, comparisonOperatorsThrowOnInvalid)
{
  ::ad::map::point::ENUCoordinate const value(::ad::map::point::ENUCoordinate::cMinValue);
  ::ad::map::point::ENUCoordinate const invalidValue;

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

TEST(ENUCoordinateTests, arithmeticOperatorsThrowOnInvalid)
{
  ::ad::map::point::ENUCoordinate const minimalValue(::ad::map::point::ENUCoordinate::cMinValue);
  ::ad::map::point::ENUCoordinate const maximalValue(::ad::map::point::ENUCoordinate::cMaxValue);
  ::ad::map::point::ENUCoordinate const invalidValue;
  ::ad::map::point::ENUCoordinate calculationValue;

  //  operator+(::ad::map::point::ENUCoordinate)
  EXPECT_THROW(invalidValue + maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue + invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue + maximalValue, std::out_of_range);

  //  operator+=(::ad::map::point::ENUCoordinate)
  calculationValue = invalidValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += invalidValue, std::out_of_range);
  calculationValue = maximalValue;
  EXPECT_THROW(calculationValue += maximalValue, std::out_of_range);

  //  operator-(::ad::map::point::ENUCoordinate)
  EXPECT_THROW(invalidValue - minimalValue, std::out_of_range);
  EXPECT_THROW(minimalValue - invalidValue, std::out_of_range);
  EXPECT_THROW(minimalValue - maximalValue, std::out_of_range);

  //  operator-=(::ad::map::point::ENUCoordinate)
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

  //  operator/(::ad::map::point::ENUCoordinate)
  EXPECT_THROW(invalidValue / maximalValue, std::out_of_range);
  EXPECT_THROW(maximalValue / invalidValue, std::out_of_range);
  EXPECT_THROW(maximalValue / ::ad::map::point::ENUCoordinate(0.0), std::out_of_range);

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

TEST(ENUCoordinateTests, comparisonOperatorsRespectPrecision)
{
  double const precisionValueTimesTen = ::ad::map::point::ENUCoordinate::cPrecisionValue * 10.;
  ::ad::map::point::ENUCoordinate value;
  if (::ad::map::point::ENUCoordinate::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::ENUCoordinate(::ad::map::point::ENUCoordinate::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::ENUCoordinate::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::ENUCoordinate(::ad::map::point::ENUCoordinate::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::ENUCoordinate(precisionValueTimesTen);
  }
  ::ad::map::point::ENUCoordinate const sameValue = value;
  ::ad::map::point::ENUCoordinate const slightlyBiggerValue(static_cast<double>(value)
                                                            + ::ad::map::point::ENUCoordinate::cPrecisionValue * 0.9);
  ::ad::map::point::ENUCoordinate const slightlySmallerValue(static_cast<double>(value)
                                                             - ::ad::map::point::ENUCoordinate::cPrecisionValue * 0.9);
  ::ad::map::point::ENUCoordinate const actuallyBiggerValue(static_cast<double>(value)
                                                            + ::ad::map::point::ENUCoordinate::cPrecisionValue * 1.1);
  ::ad::map::point::ENUCoordinate const actuallySmallerValue(static_cast<double>(value)
                                                             - ::ad::map::point::ENUCoordinate::cPrecisionValue * 1.1);

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

TEST(ENUCoordinateTests, arithmeticOperatorsComputeCorrectly)
{
  double const cDoubleNear = ::ad::map::point::ENUCoordinate::cPrecisionValue;
  double const precisionValueTimesTen = ::ad::map::point::ENUCoordinate::cPrecisionValue * 10.;
  ::ad::map::point::ENUCoordinate value;
  if (::ad::map::point::ENUCoordinate::cMinValue > precisionValueTimesTen)
  {
    value = ::ad::map::point::ENUCoordinate(::ad::map::point::ENUCoordinate::cMinValue + precisionValueTimesTen);
  }
  else if (::ad::map::point::ENUCoordinate::cMaxValue < precisionValueTimesTen)
  {
    value = ::ad::map::point::ENUCoordinate(::ad::map::point::ENUCoordinate::cMaxValue - precisionValueTimesTen);
  }
  else
  {
    value = ::ad::map::point::ENUCoordinate(precisionValueTimesTen);
  }

  ::ad::map::point::ENUCoordinate result;

  //  operator+(::ad::map::point::ENUCoordinate)
  result = value + value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator+=(::ad::map::point::ENUCoordinate)
  result = value;
  result += value;
  EXPECT_NEAR(static_cast<double>(value) + static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-(::ad::map::point::ENUCoordinate)
  result = value - value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator-=(::ad::map::point::ENUCoordinate)
  result = value;
  result -= value;
  EXPECT_NEAR(static_cast<double>(value) - static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator*(double)
  result = value * 5.;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator*(double, ::ad::map::point::ENUCoordinate)
  result = 5. * value;
  EXPECT_NEAR(static_cast<double>(value) * 5., static_cast<double>(result), cDoubleNear);

  //  operator/(double)
  result = value / static_cast<double>(value);
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), static_cast<double>(result), cDoubleNear);

  //  operator/(::ad::map::point::ENUCoordinate)
  double const doubleResult = value / value;
  EXPECT_NEAR(static_cast<double>(value) / static_cast<double>(value), doubleResult, cDoubleNear);

  //  operator-()
  if ((::ad::map::point::ENUCoordinate::cMinValue < -static_cast<double>(value))
      && (-static_cast<double>(value) < ::ad::map::point::ENUCoordinate::cMaxValue))
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
