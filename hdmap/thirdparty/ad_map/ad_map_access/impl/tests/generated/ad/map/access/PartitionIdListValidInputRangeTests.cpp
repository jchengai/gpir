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

#include "ad/map/access/PartitionIdListValidInputRange.hpp"

TEST(PartitionIdListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::access::PartitionIdList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(PartitionIdListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::access::PartitionIdList value;
  ::ad::map::access::PartitionId element(std::numeric_limits<::ad::map::access::PartitionId>::lowest());
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}
