// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

/**
 * @file
 */

#pragma once

#include "ad/physics/AngleOperation.hpp"
#include "ad/physics/ParametricOperation.hpp"
#include "ad/physics/PhysicsOperation.hpp"
#include "ad/physics/RangeOperation.hpp"
#include "ad/physics/RatioOperation.hpp"
#include "ad/physics/Types.hpp"

/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace for physics datatypes and operations
 */
namespace physics {

/*!
 * @brief ensure the physics operators defined at global scope are also found within ad::physics namespace
 */
using ::operator*;

/*!
 * @brief ensure the physics operators defined at global scope are also found within ad::physics namespace
 */
using ::operator/;

} // namespace physics
} // namespace ad
