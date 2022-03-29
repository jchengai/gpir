/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/planning_common/planning_visual.h"

#include <tf/tf.h>

#include "common/utils/common_visual.h"
#include "common/utils/str_utils.h"

namespace planning {

using common::Color;
using common::ColorMap;
using common::CommonVisual;

void PlanningVisual::FillHeader(std_msgs::Header* header) {
  header->frame_id = "map";
  header->stamp = ros::Time::now();
}

void PlanningVisual::SetScaleAndColor(const std::array<double, 3>& scale,
                                      common::Color color,
                                      visualization_msgs::Marker* marker,
                                      const double alpha) {
  marker->scale.x = scale[0];
  marker->scale.y = scale[1];
  marker->scale.z = scale[2];
  marker->color = common::ColorMap::at(color, alpha).toRosMsg();
}

void PlanningVisual::BBoxToSolidCubeMarker(const common::Box2D& bbox,
                                           const Color color,
                                           visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::CUBE;
  marker->action = visualization_msgs::Marker::MODIFY;
  marker->color = ColorMap::at(color, 0.5).toRosMsg();
  CommonVisual::StateToPose(bbox.center(), bbox.angle(), &marker->pose,
                            0.5 * bbox.height());
  marker->scale.x = bbox.length();
  marker->scale.y = bbox.width();
  marker->scale.z = bbox.height();
}

void PlanningVisual::BBoxToJskBBox(const common::Box2D& bbox,
                                   jsk_recognition_msgs::BoundingBox* jsk_bbox,
                                   const int label) {
  jsk_bbox->header.frame_id = "map";
  jsk_bbox->header.stamp = ros::Time::now();
  CommonVisual::StateToPose(bbox.center(), bbox.angle(), &jsk_bbox->pose,
                            bbox.height() / 2);
  jsk_bbox->dimensions.x = bbox.length();
  jsk_bbox->dimensions.y = bbox.width();
  jsk_bbox->dimensions.z = bbox.height();
  jsk_bbox->label = label;
}

void PlanningVisual::ObstacleToJskBBoxArray(
    const std::vector<Obstacle>& obstacles,
    jsk_recognition_msgs::BoundingBoxArray* bbox_array) {
  bbox_array->header.frame_id = "map";
  bbox_array->header.stamp = ros::Time::now();
  bbox_array->boxes.reserve(obstacles.size());

  jsk_recognition_msgs::BoundingBox bbox;
  bbox.header = bbox_array->header;
  for (const auto& obstacle : obstacles) {
    BBoxToJskBBox(obstacle.BoundingBox(), &bbox, 1);
    bbox_array->boxes.emplace_back(bbox);
  }
}

void PlanningVisual::ObstacleInfoToMarkerArray(
    const std::vector<Obstacle>& obstacles,
    visualization_msgs::MarkerArray* markers) {
  auto stamp = ros::Time::now();
  for (const auto& obstacle : obstacles) {
    int id = obstacle.id();
    auto state = obstacle.state();

    visualization_msgs::Marker marker_info;
    marker_info.header.frame_id = "map";
    marker_info.header.stamp = stamp;
    marker_info.id = 1000 * id + 1;

    marker_info.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_info.action = visualization_msgs::Marker::MODIFY;
    marker_info.color = ColorMap::at(Color::kBlack).toRosMsg();
    marker_info.scale.z = 0.5;
    marker_info.text = "ID: " + std::to_string(id) + "\n" +
                       common::to_string_with_precision(state.velocity, 2) +
                       " m/s";
    CommonVisual::PositionToPose(state.position, &marker_info.pose,
                                 obstacle.height() * 1.5);

    markers->markers.emplace_back(marker_info);
  }
}

void PlanningVisual::GetTrafficConeMarker(const Eigen::Vector2d& pos,
                                          const int id,
                                          visualization_msgs::Marker* marker) {
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time::now();
  marker->id = id;
  marker->type = visualization_msgs::Marker::MESH_RESOURCE;
  marker->action = visualization_msgs::Marker::MODIFY;
  marker->mesh_resource =
      "package://planning_core/3d_model/traffic_cone/traffic_cone.dae";

  marker->scale.x = 2.0;
  marker->scale.y = 2.0;
  marker->scale.z = 2.0;
  marker->color = ColorMap::at(Color::kRed).toRosMsg();
  marker->pose.position.x = pos.x();
  marker->pose.position.y = pos.y();
  marker->pose.position.z = 0.1;
  tf::Quaternion q(0.0, -0.7071, -0.7071, 0.0);
  tf::quaternionTFToMsg(q, marker->pose.orientation);
}

void PlanningVisual::Get2DBoxMarker(const Eigen::Vector2d& pos,
                                    const double width, const double length,
                                    const double heading, common::Color color,
                                    const std::array<double, 3>& scale,
                                    visualization_msgs::Marker* marker) {
  FillHeader(&marker->header);
  SetScaleAndColor(scale, color, marker);
  marker->type = visualization_msgs::Marker::LINE_LIST;
  marker->action = visualization_msgs::Marker::MODIFY;
  marker->pose.orientation.w = 1.0;

  const double half_length = length / 2.0 - 0.1;
  const double half_width = width / 2.0 + 0.1;

  Eigen::Vector2d tangent(std::cos(heading), std::sin(heading));
  Eigen::Vector2d normal(-std::sin(heading), std::cos(heading));
  vector_Eigen2d corners{pos + half_length * tangent + half_width * normal,
                         pos + half_length * tangent - half_width * normal,
                         pos - half_length * tangent - half_width * normal,
                         pos - half_length * tangent + half_width * normal};
  geometry_msgs::Point point;
  for (int i = 0; i < 4; ++i) {
    for (int j = i; j < i + 2; ++j) {
      int idx = j % 4;
      point.x = corners[idx].x();
      point.y = corners[idx].y();
      point.z = 0.2;
      marker->points.emplace_back(point);
    }
  }
}
}  // namespace planning