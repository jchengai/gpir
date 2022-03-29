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

#include "ad/map/access/PartitionIdValidInputRange.hpp"

TEST(PartitionIdValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::map::access::PartitionId value(0u);
  ASSERT_TRUE(withinValidInputRange(value));
}
