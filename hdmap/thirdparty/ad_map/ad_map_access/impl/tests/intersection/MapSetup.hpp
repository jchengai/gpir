// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

/* @brief namespace map_setup */
namespace map_setup {

/*
 * All function return false if the map/config could not be loaded
 * On success, the instance is initialized, and prepareMap was called
 */

void prepareMapTpkPfzDrive();
void prepareMapTrafficLightsPfz();
void prepareMapAllWayStop();
void prepareMapAllWayStopLefthand();
void prepareMapBasicPriorityToRight();
void prepareMapBasicPriorityToRightLefthand();
void prepareMapBasicPriorityToRightSingapore();
void prepareMapBasicYield();
void prepareMapBasicYieldLefthand();
void prepareMapSolidTrafficLights();
void prepareMapTown01PrioRight();
void prepareMapTown01TrafficLight();

} /* namespace map_setup */
