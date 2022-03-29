// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/access/Factory.hpp>
#include <ad/map/access/Operation.hpp>
#include <ad/map/lane/LaneOperation.hpp>
#include <ad/map/point/Operation.hpp>
#include <ad/map/serialize/SerializerFileCRC32.hpp>
#include <gtest/gtest.h>
#include "../src/access/GeometryStore.hpp"

using namespace ::ad;
using namespace ::ad::map;
using namespace ::ad::map::point;
using namespace ::ad::map::lane;

struct GeometryStoreTest : ::testing::Test
{
  GeometryStoreTest()
  {
  }

  virtual void SetUp()
  {
  }
  virtual void TearDown()
  {
    access::cleanup();
  }
};

TEST_F(GeometryStoreTest, GeometryStore)
{
  ASSERT_TRUE(access::init("test_files/Town01.txt"));
  access::GeometryStore geoStore;
  EXPECT_THROW(geoStore.store(NULL), std::runtime_error);
  EXPECT_THROW(geoStore.restore(NULL), std::runtime_error);
  EXPECT_THROW(geoStore.check(NULL), std::runtime_error);

  auto lanes = lane::getLanes();
  ASSERT_GT(lanes.size(), 0u);

  auto lanePtr = lane::getLanePtr(lanes[0]);
  ASSERT_TRUE(geoStore.store(lanePtr));
  ASSERT_TRUE(geoStore.check(lanePtr));

  lane::Lane::Ptr oneLane;
  oneLane.reset(new lane::Lane());
  oneLane->id = lanes[0];
  ASSERT_TRUE(geoStore.restore(oneLane));
  ASSERT_EQ(oneLane->edgeLeft.ecefEdge, lanePtr->edgeLeft.ecefEdge);
  ASSERT_EQ(oneLane->edgeRight.ecefEdge, lanePtr->edgeRight.ecefEdge);
}

TEST_F(GeometryStoreTest, StoreSerialization)
{
  ASSERT_TRUE(access::init("test_files/Town01.txt"));
  access::Store &store = access::getStore();
  size_t versionMajor = ::ad::map::serialize::SerializerFileCRC32::VERSION_MAJOR;
  size_t versionMinor = ::ad::map::serialize::SerializerFileCRC32::VERSION_MINOR;

  serialize::SerializerFileCRC32 serializer_w(true);
  ASSERT_TRUE(serializer_w.open("test_files/test_StoreSerialization.adm", versionMajor, versionMinor));
  ASSERT_TRUE(store.save(serializer_w, false, false, true));
  ASSERT_TRUE(serializer_w.close());

  serialize::SerializerFileCRC32 serializer_r(false);
  ASSERT_EQ(serializer_r.getStorageType(), "File");
  ASSERT_EQ(serializer_r.getChecksumType(), "CRC-32");
  ASSERT_TRUE(serializer_r.open("test_files/test_StoreSerialization.adm", versionMajor, versionMinor));
  access::Store::Ptr storeRead;
  storeRead.reset(new access::Store());
  ASSERT_TRUE(storeRead->load(serializer_r));
  ASSERT_TRUE(serializer_r.close());

  serialize::SerializerFileCRC32 serializer_w2(true);
  ASSERT_TRUE(serializer_w2.open("test_files/test_StoreSerialization.adm", versionMajor, versionMinor));
  ASSERT_TRUE(store.save(serializer_w2, false, true, true));
  ASSERT_TRUE(serializer_w2.close());

  serialize::SerializerFileCRC32 serializer_r2(false);
  ASSERT_TRUE(serializer_r2.open("test_files/test_StoreSerialization.adm", versionMajor, versionMinor));
  access::Store::Ptr storeRead2;
  storeRead2.reset(new access::Store());
  ASSERT_TRUE(storeRead2->load(serializer_r2));
  ASSERT_TRUE(serializer_r2.close());
}
