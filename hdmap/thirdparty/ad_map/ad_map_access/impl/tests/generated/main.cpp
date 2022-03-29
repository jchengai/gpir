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
#include "spdlog/sinks/null_sink.h"

int main(int argc, char **argv)
{
  auto logger = spdlog::create<spdlog::sinks::null_sink_st>("null_logger");
  spdlog::set_default_logger(logger);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
