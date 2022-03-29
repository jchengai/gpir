/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <array>

#include "hdmap/road_network/road_network.h"

namespace hdmap {

class Lane {
 public:
  explicit Lane(::ad::map::lane::Lane::ConstPtr admap_lane);
  ~Lane() = default;

  inline LaneId id() const { return id_; }
  inline double length() const { return length_; }
  inline double width() const { return width_; }

  bool HasNeighbor(const LaneNeighborType type) const;
  bool HasLeftNeighbor() const;
  bool HasRightNeighbor() const;

  bool LeftLaneChangable() const;
  bool RightLaneChangable() const;

  std::shared_ptr<const Lane> neighbor_lane(const LaneNeighborType type) const;
  std::shared_ptr<const Lane> left_lane() const;
  std::shared_ptr<const Lane> right_lane() const;

  bool HasSuccessor() const;
  bool HasSuccessor(const LaneId& id) const;
  bool HasPreSuccessor() const;
  bool HasPreSuccessor(const LaneId& id) const;

  std::vector<LaneId> next_lanes() const;
  std::vector<LaneId> previous_lanes() const;

  const std::vector<WayPoint>& way_points() const { return way_points_; }
  std::vector<WayPoint> way_points(const double start_s,
                                   const double end_s) const;
  WayPoint GetWayPoint(const double s);

  const std::vector<NeighborLane>& neighbors(const LaneNeighborType type) const;
  ad::map::lane::Lane::ConstPtr admap_lane() const { return admap_lane_; }

  double PercentageToArcLength(const double percentage_s);

  double GetArcLength(const Eigen::Vector2d& point);
  SLPair GetProjection(const Eigen::Vector2d& point);
  SLPair GetProjectionAndBoundary(const Eigen::Vector2d& point,
                                  std::pair<double, double>* boundary);

  bool IsInLane(const Eigen::Vector2d& point);

 private:
  void CreateWayPoints();
  void FindAllNeighborLanes();
  void AddNeighborLane(const LaneId neighbor_id, LaneNeighborType type);
  void AddConnectedLane(const ad::map::lane::ContactLane& contace_lane,
                        LaneConnectionType type);
  int GetNearestionPointIndex(const Eigen::Vector2d& point);
  WayPoint Interpolate(const WayPoint& p1, const WayPoint& p2,
                       const double s) const;

 private:
  LaneId id_;
  double length_ = 0.0;
  double width_ = 0.0;
  ::ad::map::lane::Lane::ConstPtr admap_lane_;

  std::array<std::vector<NeighborLane>, 2> neighbor_lanes_;
  std::array<std::vector<ConnectedLane>, 2> connected_lanes_;

  int points_num_;
  std::vector<WayPoint> way_points_;
  std::vector<std::pair<double, double>> lane_boundary_;

  friend class HdMapImpl;
};

// inline
}  // namespace hdmap