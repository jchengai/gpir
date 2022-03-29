// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/serialize/SerializerFileCRC32.hpp>
#include <gtest/gtest.h>

#include "../access/FactoryTests.hpp"

using namespace ::ad::map::serialize;

struct SerializationTest : public FactoryTest
{
  SerializationTest()
  {
  }

  bool compareStores(access::Store const &left, access::Store const &right)
  {
    if (left.isValid() && right.isValid())
    {
      bool equal = (left.getMetaData() == right.getMetaData()) && (left.getPartitions() == right.getPartitions())
        && (left.getLanes() == right.getLanes()) && (left.getLandmarks() == right.getLandmarks());

      for (auto laneId : left.getLanes())
      {
        equal = equal && *left.getLanePtr(laneId) == *right.getLanePtr(laneId);
      }
      for (auto landmarkId : left.getLandmarks())
      {
        equal = equal && *left.getLandmarkPtr(landmarkId) == *right.getLandmarkPtr(landmarkId);
      }
      return equal;
    }
    else
    {
      return false;
    }
  }

  template <typename SerializationFile> void TestSerializationFile(const char *test_file_name)
  {
    size_t version_major_write = ::ad::map::serialize::SerializerFileCRC32::VERSION_MAJOR;
    size_t version_minor_write = ::ad::map::serialize::SerializerFileCRC32::VERSION_MINOR;

    SerializationFile serializer(true);
    ASSERT_TRUE(serializer.open(test_file_name, version_major_write, version_minor_write));
    ASSERT_TRUE(mStorePtr->save(serializer));
    ASSERT_TRUE(serializer.close());

    size_t version_major_read = 0xBAD;
    size_t version_minor_read = 0xBAD;

    SerializationFile deserializer(false);
    ASSERT_TRUE(deserializer.open(test_file_name, version_major_read, version_minor_read));
    ASSERT_EQ(version_major_write, version_major_read);
    ASSERT_EQ(version_minor_read, version_minor_write);
    access::Store::Ptr readStore(new access::Store());
    ASSERT_TRUE(readStore->load(deserializer));
    ASSERT_TRUE(deserializer.close());

    ASSERT_TRUE(compareStores(*mStorePtr, *readStore));
  }
};

TEST_F(SerializationTest, TestSerializationFileCRC32)
{
  TestSerializationFile<SerializerFileCRC32>("test_files/test_serialization_crc32.adm");
}

TEST_F(SerializationTest, TestMapVersion)
{
  size_t version_major = 0;
  size_t version_minor = 0;
  // read the map
  SerializerFileCRC32 deserializer(false);
  ASSERT_THROW(deserializer.open("test_files/testmap_wrong_version.adm", version_major, version_minor),
               std::runtime_error);

  access::Store::Ptr store(new access::Store());

  // write the map
  SerializerFileCRC32 serializer(true);
  serializer.open("test_files/test_map_version_correct.adm", version_major, version_minor);
  store->save(serializer);
  serializer.close();

  SerializerFileCRC32 deserializer_correct(false);
  ASSERT_TRUE(deserializer_correct.open("test_files/test_map_version_correct.adm", version_major, version_minor));
  ASSERT_TRUE(store->load(deserializer_correct));
  ASSERT_TRUE(deserializer_correct.close());

  // check the version number
  ASSERT_EQ(::ad::map::serialize::SerializerFileCRC32::VERSION_MAJOR, version_major);
  ASSERT_EQ(::ad::map::serialize::SerializerFileCRC32::VERSION_MINOR, version_minor);
}

TEST_F(SerializationTest, BadBranch)
{
  size_t version_major = 0;
  size_t version_minor = 0;
  access::Store::Ptr store0(new access::Store());
  access::Store::Ptr store1(new access::Store());

  uint16_t temp = 0;
  SerializerFileCRC32 serializerW(true);
  ASSERT_FALSE(serializerW.close());
  ASSERT_FALSE(serializerW.serialize(temp));
  ASSERT_TRUE(serializerW.open("test_files/test_map_version_correct.adm", version_major, version_minor));
  ASSERT_FALSE(serializerW.open("test_files/test_map_version_correct.adm", version_major, version_minor));
  ASSERT_FALSE(store0->load(serializerW));
  ASSERT_TRUE(store0->save(serializerW));
  ASSERT_TRUE(serializerW.close());
  ASSERT_FALSE(serializerW.close());

  SerializerFileCRC32 serializerR(false);
  ASSERT_FALSE(serializerR.close());
  ASSERT_FALSE(serializerR.serialize(temp));
  ASSERT_TRUE(serializerR.open("test_files/test_map_version_correct.adm", version_major, version_minor));
  ASSERT_FALSE(serializerR.open("test_files/test_map_version_correct.adm", version_major, version_minor));
  ASSERT_FALSE(store1->save(serializerR));
  ASSERT_TRUE(store1->load(serializerR));
  ASSERT_TRUE(serializerR.close());
  ASSERT_FALSE(serializerR.close());
}
