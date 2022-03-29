// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "MapSetup.hpp"

#include <ad/map/access/Operation.hpp>
#include <ad/map/point/Operation.hpp>
#include <stdexcept>

/* @brief namespace map_setup */
namespace map_setup {

using namespace ::ad::map::point;

void prepareMap(const char *filename)
{
  ::ad::map::access::cleanup();
  if (!ad::map::access::init(filename))
  {
    throw std::runtime_error("Unable to load config " + std::string(filename));
  }
}

void prepareMapAllWayStop()
{
  prepareMap("test_files/AllWayStop.adm.txt");
}

void prepareMapAllWayStopLefthand()
{
  prepareMap("test_files/AllWayStop.lefthand.adm.txt");
}

void prepareMapBasicPriorityToRight()
{
  prepareMap("test_files/BasicPriorityToRight.adm.txt");
}

void prepareMapBasicPriorityToRightLefthand()
{
  prepareMap("test_files/BasicPriorityToRight.lefthand.adm.txt");
}

void prepareMapBasicPriorityToRightSingapore()
{
  prepareMap("test_files/BasicPriorityToRight.singapore.adm.txt");
}

void prepareMapBasicYield()
{
  prepareMap("test_files/BasicYield.adm.txt");
}

void prepareMapBasicYieldLefthand()
{
  prepareMap("test_files/BasicYield.lefthand.adm.txt");
}

void prepareMapTrafficLightsPfz()
{
  prepareMap("test_files/PFZ_Traffic_Lights.adm.txt");
}

void prepareMapTpkPfzDrive()
{
  prepareMap("test_files/TPK_PFZ.adm.txt");
}

void prepareMapSolidTrafficLights()
{
  prepareMap("test_files/SolidTrafficLights.adm.txt");
}

void prepareMapTown01PrioRight()
{
  prepareMap("test_files/Town01.prioRight.txt");
}

void prepareMapTown01TrafficLight()
{
  prepareMap("test_files/Town01.txt");
}

} /* namespace map_setup */
