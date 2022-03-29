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

#include "ad/map/lane/ContactLocationValidInputRange.hpp"

TEST(ContactLocationValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactLocation::INVALID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactLocation::UNKNOWN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactLocation::LEFT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactLocation::RIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactLocation::SUCCESSOR));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactLocation::PREDECESSOR));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactLocation::OVERLAP));
}

TEST(ContactLocationValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::INVALID));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::UNKNOWN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::LEFT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::RIGHT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::SUCCESSOR));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::PREDECESSOR));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::OVERLAP));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::lane::ContactLocation>(minValue - 1)));
}
