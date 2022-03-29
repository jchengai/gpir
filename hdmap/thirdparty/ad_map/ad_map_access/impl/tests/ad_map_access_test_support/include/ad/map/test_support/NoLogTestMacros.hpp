// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

// clang-format off

#define ASSERT_FALSE_NO_LOG(arg) do { \
  auto currentLogLevel = spdlog::default_logger()->level(); \
  spdlog::set_level(spdlog::level::off); \
  ASSERT_FALSE(arg); \
  spdlog::set_level(currentLogLevel); \
} while(0)

#define ASSERT_TRUE_NO_LOG(arg) do { \
  auto currentLogLevel = spdlog::default_logger()->level(); \
  spdlog::set_level(spdlog::level::off); \
  ASSERT_TRUE(arg); \
  spdlog::set_level(currentLogLevel); \
} while(0)

#define EXPECT_THROW_NO_LOG(arg1, arg2) do { \
  auto currentLogLevel = spdlog::default_logger()->level(); \
  spdlog::set_level(spdlog::level::off); \
  EXPECT_THROW(arg1, arg2); \
  spdlog::set_level(currentLogLevel); \
} while(0)

#define EXPECT_TRUE_NO_LOG(arg) do { \
  auto currentLogLevel = spdlog::default_logger()->level(); \
  spdlog::set_level(spdlog::level::off); \
  EXPECT_TRUE(arg); \
  spdlog::set_level(currentLogLevel); \
} while(0)

#define EXPECT_FALSE_NO_LOG(arg) do { \
  auto currentLogLevel = spdlog::default_logger()->level(); \
  spdlog::set_level(spdlog::level::off); \
  EXPECT_FALSE(arg); \
  spdlog::set_level(currentLogLevel); \
} while(0)

#define EXECUTE_NO_LOG(arg) do { \
  auto currentLogLevel = spdlog::default_logger()->level(); \
  spdlog::set_level(spdlog::level::off); \
  arg; \
  spdlog::set_level(currentLogLevel); \
} while(0)

// clang-format on
