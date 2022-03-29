// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/access/Logging.hpp"
#include "AdMapAccess.hpp"

namespace ad {
namespace map {
namespace access {

std::shared_ptr<spdlog::logger> getLogger()
{
  return AdMapAccess::getAdMapAccessInstance().mLogger;
}

} // namespace access
} // namespace map
} // namespace ad
