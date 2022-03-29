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

#include "ad/map/lane/ContactType.hpp"

#include <stdexcept>

std::string toString(::ad::map::lane::ContactType const e) {
  switch (e) {
    case ::ad::map::lane::ContactType::INVALID:
      return std::string("INVALID");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::UNKNOWN:
      return std::string("UNKNOWN");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::FREE:
      return std::string("FREE");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::LANE_CHANGE:
      return std::string("LANE_CHANGE");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::LANE_CONTINUATION:
      return std::string("LANE_CONTINUATION");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::LANE_END:
      return std::string("LANE_END");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::SINGLE_POINT:
      return std::string("SINGLE_POINT");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::STOP:
      return std::string("STOP");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::STOP_ALL:
      return std::string("STOP_ALL");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::YIELD:
      return std::string("YIELD");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::GATE_BARRIER:
      return std::string("GATE_BARRIER");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::GATE_TOLBOOTH:
      return std::string("GATE_TOLBOOTH");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::GATE_SPIKES:
      return std::string("GATE_SPIKES");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::GATE_SPIKES_CONTRA:
      return std::string("GATE_SPIKES_CONTRA");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::CURB_UP:
      return std::string("CURB_UP");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::CURB_DOWN:
      return std::string("CURB_DOWN");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::SPEED_BUMP:
      return std::string("SPEED_BUMP");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::TRAFFIC_LIGHT:
      return std::string("TRAFFIC_LIGHT");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::CROSSWALK:
      return std::string("CROSSWALK");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::PRIO_TO_RIGHT:
      return std::string("PRIO_TO_RIGHT");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::RIGHT_OF_WAY:
      return std::string("RIGHT_OF_WAY");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT:
      return std::string("PRIO_TO_RIGHT_AND_STRAIGHT");  // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE");  // LCOV_EXCL_BR_LINE
  }
}

template <>
::ad::map::lane::ContactType fromString(std::string const &str) {
  if (str == std::string("INVALID"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::INVALID;
  }
  if (str == std::string("INVALID"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::INVALID;
  }
  if (str == std::string("UNKNOWN"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::UNKNOWN;
  }
  if (str == std::string("UNKNOWN"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::UNKNOWN;
  }
  if (str == std::string("FREE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::FREE;
  }
  if (str == std::string("FREE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::FREE;
  }
  if (str == std::string("LANE_CHANGE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::LANE_CHANGE;
  }
  if (str == std::string("LANE_CHANGE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::LANE_CHANGE;
  }
  if (str == std::string("LANE_CONTINUATION"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::LANE_CONTINUATION;
  }
  if (str == std::string("LANE_CONTINUATION"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::LANE_CONTINUATION;
  }
  if (str == std::string("LANE_END"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::LANE_END;
  }
  if (str == std::string("LANE_END"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::LANE_END;
  }
  if (str == std::string("SINGLE_POINT"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::SINGLE_POINT;
  }
  if (str == std::string("SINGLE_POINT"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::SINGLE_POINT;
  }
  if (str == std::string("STOP"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::STOP;
  }
  if (str == std::string("STOP"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::STOP;
  }
  if (str == std::string("STOP_ALL"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::STOP_ALL;
  }
  if (str == std::string("STOP_ALL"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::STOP_ALL;
  }
  if (str == std::string("YIELD"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::YIELD;
  }
  if (str == std::string("YIELD"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::YIELD;
  }
  if (str == std::string("GATE_BARRIER"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::GATE_BARRIER;
  }
  if (str == std::string("GATE_BARRIER"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::GATE_BARRIER;
  }
  if (str == std::string("GATE_TOLBOOTH"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::GATE_TOLBOOTH;
  }
  if (str == std::string("GATE_TOLBOOTH"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::GATE_TOLBOOTH;
  }
  if (str == std::string("GATE_SPIKES"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::GATE_SPIKES;
  }
  if (str == std::string("GATE_SPIKES"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::GATE_SPIKES;
  }
  if (str == std::string("GATE_SPIKES_CONTRA"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::GATE_SPIKES_CONTRA;
  }
  if (str == std::string("GATE_SPIKES_CONTRA"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::GATE_SPIKES_CONTRA;
  }
  if (str == std::string("CURB_UP"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::CURB_UP;
  }
  if (str == std::string("CURB_UP"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::CURB_UP;
  }
  if (str == std::string("CURB_DOWN"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::CURB_DOWN;
  }
  if (str == std::string("CURB_DOWN"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::CURB_DOWN;
  }
  if (str == std::string("SPEED_BUMP"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::SPEED_BUMP;
  }
  if (str == std::string("SPEED_BUMP"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::SPEED_BUMP;
  }
  if (str == std::string("TRAFFIC_LIGHT"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::TRAFFIC_LIGHT;
  }
  if (str == std::string("TRAFFIC_LIGHT"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::TRAFFIC_LIGHT;
  }
  if (str == std::string("CROSSWALK"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::CROSSWALK;
  }
  if (str == std::string("CROSSWALK"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::CROSSWALK;
  }
  if (str == std::string("PRIO_TO_RIGHT"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::PRIO_TO_RIGHT;
  }
  if (str == std::string("PRIO_TO_RIGHT"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::PRIO_TO_RIGHT;
  }
  if (str == std::string("RIGHT_OF_WAY"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::RIGHT_OF_WAY;
  }
  if (str == std::string("RIGHT_OF_WAY"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::RIGHT_OF_WAY;
  }
  if (str == std::string("PRIO_TO_RIGHT_AND_"
                         "STRAIGHT"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT;
  }
  if (str == std::string("PRIO_TO_RIGHT_AND_STRAIGHT"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT;
  }
  throw std::out_of_range("Invalid enum literal");  // LCOV_EXCL_BR_LINE
}
