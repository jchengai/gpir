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
#include "ad/map/lane/ContactLocation.hpp"

TEST(ContactLocationTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("INVALID"), ::ad::map::lane::ContactLocation::INVALID);
  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("::ad::map::lane::ContactLocation::INVALID"),
            ::ad::map::lane::ContactLocation::INVALID);

  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("UNKNOWN"), ::ad::map::lane::ContactLocation::UNKNOWN);
  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("::ad::map::lane::ContactLocation::UNKNOWN"),
            ::ad::map::lane::ContactLocation::UNKNOWN);

  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("LEFT"), ::ad::map::lane::ContactLocation::LEFT);
  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("::ad::map::lane::ContactLocation::LEFT"),
            ::ad::map::lane::ContactLocation::LEFT);

  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("RIGHT"), ::ad::map::lane::ContactLocation::RIGHT);
  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("::ad::map::lane::ContactLocation::RIGHT"),
            ::ad::map::lane::ContactLocation::RIGHT);

  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("SUCCESSOR"), ::ad::map::lane::ContactLocation::SUCCESSOR);
  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("::ad::map::lane::ContactLocation::SUCCESSOR"),
            ::ad::map::lane::ContactLocation::SUCCESSOR);

  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("PREDECESSOR"), ::ad::map::lane::ContactLocation::PREDECESSOR);
  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("::ad::map::lane::ContactLocation::PREDECESSOR"),
            ::ad::map::lane::ContactLocation::PREDECESSOR);

  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("OVERLAP"), ::ad::map::lane::ContactLocation::OVERLAP);
  ASSERT_EQ(fromString<::ad::map::lane::ContactLocation>("::ad::map::lane::ContactLocation::OVERLAP"),
            ::ad::map::lane::ContactLocation::OVERLAP);

  EXPECT_ANY_THROW({ fromString<::ad::map::lane::ContactLocation>("NOT A VALID ENUM LITERAL"); });
}

TEST(ContactLocationTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::lane::ContactLocation::INVALID), "::ad::map::lane::ContactLocation::INVALID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::INVALID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::INVALID));

  ASSERT_EQ(toString(::ad::map::lane::ContactLocation::UNKNOWN), "::ad::map::lane::ContactLocation::UNKNOWN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::UNKNOWN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::UNKNOWN));

  ASSERT_EQ(toString(::ad::map::lane::ContactLocation::LEFT), "::ad::map::lane::ContactLocation::LEFT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::LEFT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::LEFT));

  ASSERT_EQ(toString(::ad::map::lane::ContactLocation::RIGHT), "::ad::map::lane::ContactLocation::RIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::RIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::RIGHT));

  ASSERT_EQ(toString(::ad::map::lane::ContactLocation::SUCCESSOR), "::ad::map::lane::ContactLocation::SUCCESSOR");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::SUCCESSOR));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::SUCCESSOR));

  ASSERT_EQ(toString(::ad::map::lane::ContactLocation::PREDECESSOR), "::ad::map::lane::ContactLocation::PREDECESSOR");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::PREDECESSOR));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::PREDECESSOR));

  ASSERT_EQ(toString(::ad::map::lane::ContactLocation::OVERLAP), "::ad::map::lane::ContactLocation::OVERLAP");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::OVERLAP));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactLocation::OVERLAP));

  ASSERT_EQ(toString(static_cast<::ad::map::lane::ContactLocation>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::lane::ContactLocation>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(ContactLocationTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::lane::ContactLocation value(::ad::map::lane::ContactLocation::INVALID);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
