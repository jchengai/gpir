/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2018-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

/**
 * Generated file
 * @file
 *
 * Generator Version : 11.0.0-1997
 */

#pragma once

#include <iostream>
#include <sstream>
#include <vector>

#include "ad/map/lane/LaneId.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace lane
 *
 * Handling of lanes
 */
namespace lane {

/*!
 * \brief DataType LaneIdList
 *
 * List of lane identifiers
 */
typedef std::vector<::ad::map::lane::LaneId> LaneIdList;

}  // namespace lane
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_VECTOR_AD_MAP_LANE_LANEID
#define GEN_GUARD_VECTOR_AD_MAP_LANE_LANEID
namespace std {
/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value LaneIdList value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os,
                                vector<::ad::map::lane::LaneId> const &_value) {
  os << "[";
  for (auto it = _value.begin(); it != _value.end(); it++) {
    if (it != _value.begin()) {
      os << ",";
    }
    os << *it;
  }
  os << "]";
  return os;
}
}  // namespace std

namespace std {
/*!
 * \brief overload of the std::to_string for LaneIdList
 */
inline std::string to_string(::ad::map::lane::LaneIdList const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_VECTOR_AD_MAP_LANE_LANEID
