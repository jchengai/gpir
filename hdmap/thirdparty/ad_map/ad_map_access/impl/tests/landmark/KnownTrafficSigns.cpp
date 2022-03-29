// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "KnownTrafficSigns.hpp"

namespace ad {
namespace map {
namespace landmark {

std::vector<TrafficSignType> knownTrafficSigns = {TrafficSignType::INVALID,
                                                  TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT,
                                                  TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT,
                                                  TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT,
                                                  TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN,
                                                  TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE,
                                                  TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE,
                                                  TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME,
                                                  TrafficSignType::SUPPLEMENT_ENDS,
                                                  TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED,
                                                  TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED,
                                                  TrafficSignType::SUPPLEMENT_MOPED_ALLOWED,
                                                  TrafficSignType::SUPPLEMENT_TRAM_ALLOWED,
                                                  TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED,
                                                  TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED,
                                                  TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN,
                                                  TrafficSignType::SUPPLEMENT_RAILWAY_ONLY,
                                                  TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT,
                                                  TrafficSignType::DANGER,
                                                  TrafficSignType::LANES_MERGING,
                                                  TrafficSignType::CAUTION_PEDESTRIAN,
                                                  TrafficSignType::CAUTION_CHILDREN,
                                                  TrafficSignType::CAUTION_BICYCLE,
                                                  TrafficSignType::CAUTION_ANIMALS,
                                                  TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER,
                                                  TrafficSignType::CAUTION_RAIL_CROSSING,
                                                  TrafficSignType::YIELD_TRAIN,
                                                  TrafficSignType::YIELD,
                                                  TrafficSignType::STOP,
                                                  TrafficSignType::REQUIRED_RIGHT_TURN,
                                                  TrafficSignType::REQUIRED_LEFT_TURN,
                                                  TrafficSignType::REQUIRED_STRAIGHT,
                                                  TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN,
                                                  TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN,
                                                  TrafficSignType::ROUNDABOUT,
                                                  TrafficSignType::PASS_RIGHT,
                                                  TrafficSignType::PASS_LEFT,
                                                  TrafficSignType::BYBICLE_PATH,
                                                  TrafficSignType::FOOTWALK,
                                                  TrafficSignType::FOOTWALK_BICYCLE_SHARED,
                                                  TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT,
                                                  TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT,
                                                  TrafficSignType::PEDESTRIAN_AREA_BEGIN,
                                                  TrafficSignType::ACCESS_FORBIDDEN,
                                                  TrafficSignType::ACCESS_FORBIDDEN_TRUCKS,
                                                  TrafficSignType::ACCESS_FORBIDDEN_BICYCLE,
                                                  TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES,
                                                  TrafficSignType::ACCESS_FORBIDDEN_WEIGHT,
                                                  TrafficSignType::ACCESS_FORBIDDEN_WIDTH,
                                                  TrafficSignType::ACCESS_FORBIDDEN_HEIGHT,
                                                  TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR,
                                                  TrafficSignType::ENVIORNMENT_ZONE_BEGIN,
                                                  TrafficSignType::ENVIORNMENT_ZONE_END,
                                                  TrafficSignType::MAX_SPEED,
                                                  TrafficSignType::SPEED_ZONE_30_BEGIN,
                                                  TrafficSignType::SPEED_ZONE_30_END,
                                                  TrafficSignType::HAS_WAY_NEXT_INTERSECTION,
                                                  TrafficSignType::PRIORITY_WAY,
                                                  TrafficSignType::CITY_BEGIN,
                                                  TrafficSignType::CITY_END,
                                                  TrafficSignType::MOTORWAY_BEGIN,
                                                  TrafficSignType::MOTORWAY_END,
                                                  TrafficSignType::MOTORVEHICLE_BEGIN,
                                                  TrafficSignType::MOTORVEHICLE_END,
                                                  TrafficSignType::INFO_MOTORWAY_INFO,
                                                  TrafficSignType::CUL_DE_SAC,
                                                  TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE,
                                                  TrafficSignType::INFO_NUMBER_OF_AUTOBAHN,
                                                  TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN,
                                                  TrafficSignType::DIRECTION_TURN_TO_LOCAL,
                                                  TrafficSignType::DESTINATION_BOARD,
                                                  TrafficSignType::FREE_TEXT,
                                                  TrafficSignType::UNKNOWN};

} // namespace landmark
} // namespace map
} // namespace ad
