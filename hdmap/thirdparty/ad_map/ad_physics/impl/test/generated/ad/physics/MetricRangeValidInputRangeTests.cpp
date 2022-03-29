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

#include "ad/physics/MetricRangeValidInputRange.hpp"

TEST(MetricRangeValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::MetricRange value;
  ::ad::physics::Distance valueMinimum(-1e9);
  valueMinimum = ::ad::physics::Distance(0.); // set to valid value within struct
  value.minimum = valueMinimum;
  ::ad::physics::Distance valueMaximum(-1e9);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(MetricRangeValidInputRangeTests, testValidInputRangeMinimumTooSmall)
{
  ::ad::physics::MetricRange value;
  ::ad::physics::Distance valueMinimum(-1e9);
  valueMinimum = ::ad::physics::Distance(0.); // set to valid value within struct
  value.minimum = valueMinimum;
  ::ad::physics::Distance valueMaximum(-1e9);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));

  // override member with value below struct member input range minimum
  invalidInitializedMember
    = ::ad::physics::Distance(0. - ::ad::physics::Distance::cPrecisionValue); // set to invalid value within struct
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MetricRangeValidInputRangeTests, testValidInputRangeMinimumTooBig)
{
  ::ad::physics::MetricRange value;
  ::ad::physics::Distance valueMinimum(-1e9);
  valueMinimum = ::ad::physics::Distance(0.); // set to valid value within struct
  value.minimum = valueMinimum;
  ::ad::physics::Distance valueMaximum(-1e9);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MetricRangeValidInputRangeTests, testValidInputRangeminimumDefault)
{
  ::ad::physics::MetricRange value;
  ::ad::physics::Distance valueDefault;
  value.minimum = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MetricRangeValidInputRangeTests, testValidInputRangeMaximumTooSmall)
{
  ::ad::physics::MetricRange value;
  ::ad::physics::Distance valueMinimum(-1e9);
  valueMinimum = ::ad::physics::Distance(0.); // set to valid value within struct
  value.minimum = valueMinimum;
  ::ad::physics::Distance valueMaximum(-1e9);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MetricRangeValidInputRangeTests, testValidInputRangeMaximumTooBig)
{
  ::ad::physics::MetricRange value;
  ::ad::physics::Distance valueMinimum(-1e9);
  valueMinimum = ::ad::physics::Distance(0.); // set to valid value within struct
  value.minimum = valueMinimum;
  ::ad::physics::Distance valueMaximum(-1e9);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));

  // override member with value above struct member input range maximum
  invalidInitializedMember = ::ad::physics::Distance(1e6 * 1.1); // set to invalid value within struct
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MetricRangeValidInputRangeTests, testValidInputRangemaximumDefault)
{
  ::ad::physics::MetricRange value;
  ::ad::physics::Distance valueDefault;
  value.maximum = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
