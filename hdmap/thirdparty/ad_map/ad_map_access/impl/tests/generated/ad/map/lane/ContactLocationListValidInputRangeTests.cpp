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

#include "ad/map/lane/ContactLocationListValidInputRange.hpp"

TEST(ContactLocationListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::lane::ContactLocationList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ContactLocationListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::lane::ContactLocationList value;
  ::ad::map::lane::ContactLocation element(::ad::map::lane::ContactLocation::INVALID);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ContactLocationListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::lane::ContactLocationList value;
  ::ad::map::lane::ContactLocation element(static_cast<::ad::map::lane::ContactLocation>(-1));
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
