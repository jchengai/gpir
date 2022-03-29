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

#include "ad/map/lane/ContactTypeListValidInputRange.hpp"

TEST(ContactTypeListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::lane::ContactTypeList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ContactTypeListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::lane::ContactTypeList value;
  ::ad::map::lane::ContactType element(::ad::map::lane::ContactType::INVALID);
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ContactTypeListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::lane::ContactTypeList value;
  ::ad::map::lane::ContactType element(static_cast<::ad::map::lane::ContactType>(-1));
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
