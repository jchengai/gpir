// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <spdlog/logger.h>

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace access */
namespace access {

/**
 * @brief get the admap logger
 */

std::shared_ptr<spdlog::logger> getLogger();

} // namespace access
} // namespace map
} // namespace ad
