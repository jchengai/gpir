// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <ad/map/access/Factory.hpp>
#include <ad/map/access/Operation.hpp>
#include <ad/map/landmark/LandmarkOperation.hpp>
#include <ad/map/lane/ContactOperation.hpp>
#include <ad/map/lane/LaneOperation.hpp>
#include <ad/map/point/PointOperation.hpp>
#include <gtest/gtest.h>
#include "../point/RandomGeometry.hpp"

using namespace ::ad;
using namespace ::ad::map;

struct FactoryTest : ::testing::Test
{
  FactoryTest();
  virtual void SetUp();
  virtual void TearDown();

  bool Fill();

  void Check(const lane::LaneId &id, lane::LaneType type, lane::LaneDirection direction);
  void Check(const lane::LaneId &id, point::Geometry const &edgeLeft, point::Geometry const &edgeRight);
  void Check(const lane::LaneId &id_from,
             const lane::LaneId &id_to,
             lane::ContactLocation location,
             const lane::ContactTypeList &types);
  bool IsAccessOk(const lane::LaneId &id, restriction::RoadUserType road_user_type);
  bool IsTransitionOk(const lane::LaneId &id_from,
                      const lane::LaneId &id_to,
                      lane::ContactLocation location,
                      restriction::RoadUserType road_user_type);

  restriction::VehicleDescriptor mVehicle;
  access::Store::Ptr mStorePtr;
  std::shared_ptr<access::Factory> pFactory;
  lane::LaneId x11{11}, x12{12}, x13{13}, x100{100}, xInValid;
  point::Geometry edgeLeft11, edgeRight11;
  point::Geometry edgeLeft12, edgeRight12;
  point::Geometry edgeLeft13, edgeRight13;
};
