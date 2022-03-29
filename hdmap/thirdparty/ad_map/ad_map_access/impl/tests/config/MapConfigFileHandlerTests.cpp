// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/config/MapConfigFileHandler.hpp>
#include <ad/map/test_support/NoLogTestMacros.hpp>

#include <gtest/gtest.h>

TEST(MapConfigFileHandler, readValidConfig)
{
  ad::map::config::MapConfigFileHandler configHandler{};
  ASSERT_STREQ("", configHandler.configFileName().c_str());
  ASSERT_TRUE(configHandler.readConfig("test_files/TPK.adm.txt"));
  ASSERT_TRUE(configHandler.isInitialized());

  // the configFileName is a fully qualified path, check that it is correctly returned
  auto const &actual = configHandler.configFileName();
  std::string const expected("TPK.adm.txt");
  ASSERT_GT(actual.size(), expected.size());
  auto expectedPos = actual.size() - expected.size();
  ASSERT_EQ(actual.find(expected), expectedPos);

  EXPECT_EQ(6u, configHandler.pointsOfInterest().size());

  ASSERT_FALSE_NO_LOG(configHandler.readConfig("test_files/non_existing.txt"));
  // reading a second time
  ASSERT_TRUE(configHandler.readConfig("test_files/TPK.adm.txt"));
}

TEST(MapConfigFileHandler, non_existing)
{
  ad::map::config::MapConfigFileHandler configHandler{};
  ASSERT_FALSE_NO_LOG(configHandler.readConfig("test_files/non_existing.txt"));
  ASSERT_FALSE(configHandler.isInitialized());
  ASSERT_STREQ("", configHandler.configFileName().c_str());
  ASSERT_TRUE(configHandler.readConfig("test_files/TPK.adm.txt"));
  ASSERT_TRUE(configHandler.isInitialized());
}

TEST(MapConfigFileHandler, map_config_invalid_map_location)
{
  ad::map::config::MapConfigFileHandler configHandler{};
  ASSERT_FALSE_NO_LOG(configHandler.readConfig("test_files/map_config_invalid_map_location.txt"));
}

TEST(MapConfigFileHandler, map_config_map_location_out_of_tree)
{
  ad::map::config::MapConfigFileHandler configHandler{};
  ASSERT_FALSE_NO_LOG(configHandler.readConfig("test_files/map_config_map_location_out_of_tree.txt"));
}

TEST(MapConfigFileHandler, invalid_default_enu_reference_config)
{
  ad::map::config::MapConfigFileHandler configHandler{};
  ASSERT_FALSE_NO_LOG(configHandler.readConfig("test_files/map_config_invalid_default_enu_reference.txt"));
  EXPECT_FALSE(configHandler.defaultEnuReferenceAvailable());
}

TEST(MapConfigFileHandler, valid_default_enu_reference_config)
{
  ad::map::config::MapConfigFileHandler configHandler{};
  ASSERT_TRUE(configHandler.readConfig("test_files/TPK_PFZ.adm.txt"));
  EXPECT_TRUE(configHandler.defaultEnuReferenceAvailable());
  EXPECT_EQ(ad::map::point::Latitude(49.0080515), configHandler.defaultEnuReference().latitude);
  EXPECT_EQ(ad::map::point::Longitude(8.442078), configHandler.defaultEnuReference().longitude);
  EXPECT_EQ(ad::map::point::Altitude(0.), configHandler.defaultEnuReference().altitude);
}

TEST(MapConfigFileHandler, reset)
{
  ad::map::config::MapConfigFileHandler configHandler{};
  ASSERT_STREQ("", configHandler.configFileName().c_str());
  ASSERT_TRUE(configHandler.readConfig("test_files/TPK.adm.txt"));
  ASSERT_TRUE(configHandler.isInitialized());

  configHandler.reset();
  ASSERT_FALSE(configHandler.isInitialized());
  ASSERT_EQ(configHandler.adMapEntry().filename, "");
  ASSERT_TRUE(configHandler.configFileName().empty());
  ASSERT_TRUE(configHandler.pointsOfInterest().empty());
}

TEST(MapConfigFileHandler, bad_branch)
{
  ad::map::config::MapConfigFileHandler configHandler{};
  ASSERT_STREQ("", configHandler.configFileName().c_str());
  ASSERT_FALSE(configHandler.readConfig("test_files/non_existent.txt"));
  ASSERT_FALSE(configHandler.readConfig("test_files/bad/poi.bad1.txt"));
  ASSERT_FALSE(configHandler.readConfig("test_files/bad/poi.bad2.txt"));
  ASSERT_FALSE(configHandler.readConfig("test_files/bad/poi.bad3.txt"));
  ASSERT_FALSE(configHandler.readConfig("test_files/bad/poi.bad4.txt"));
  ASSERT_FALSE(configHandler.readConfig("test_files/bad/poi.bad5.txt"));

  ASSERT_FALSE(configHandler.readConfig("test_files/bad/enu.bad1.txt"));
  ASSERT_FALSE(configHandler.readConfig("test_files/bad/enu.bad2.txt"));
  ASSERT_FALSE(configHandler.readConfig("test_files/bad/enu.bad3.txt"));

  ASSERT_FALSE(configHandler.readConfig("test_files/bad/opendrive.overlap.bad.txt"));
}
