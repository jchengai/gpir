/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include "ad/map/access/Logging.hpp"

int main(int argc, char **argv)
{
  spdlog::set_default_logger(::ad::map::access::getLogger());

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
