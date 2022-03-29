// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <utility>
#include <vector>

#include "ad/map/point/ECEFOperation.hpp"
#include "ad/map/point/ENUOperation.hpp"
#include "ad/map/point/PointOperation.hpp"
#include "ad/physics/MetricRange.hpp"
#include "ad/physics/Operation.hpp"
#include "ad/physics/ParametricRange.hpp"
#include "ad/physics/ParametricValueValidInputRange.hpp"
#include "ad/physics/RangeOperation.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace point */
namespace point {

/**
 * @brief The edge point border distance
 */
physics::Distance const cEdgePointBorderDistance{0.1};

/**
 * @brief Find point nearest to the line formed by two points.
 *
 * @param[in] a point to search for
 * @param[in] pt0 First point of the line.
 * @param[in] pt1 Second point of the line.
 * @returns Value of t;  nearest point to a can be calculated as
 * (1-t)*pt0+t*pt1.
 */
template <typename PointType>
physics::RatioValue findNearestPointOnEdge(PointType const &a,
                                           const PointType &pt0,
                                           const PointType &pt1) {
  PointType line = pt1 - pt0;
  PointType pt02a = a - pt0;
  auto const divisor = vectorDotProduct(line, line);
  if (physics::Distance(divisor) > physics::Distance(0)) {
    auto const dividend = vectorDotProduct(pt02a, line);
    auto result = dividend / divisor;
    return physics::RatioValue(result);
  } else {
    return physics::RatioValue(0.5);
  }
}

/**
 * @brief Find point nearest to the segment formed by two points.
 *
 * @param[in] a point to search for
 * @param[in] pt0 First point of the segment.
 * @param[in] pt1 Second point of the segment.
 * @returns Value of 0<=t<=1;  nearest point to this can be calculated as
 * (1-t)*pt0+t*pt1.
 *
 */
template <typename PointType>
physics::ParametricValue findNearestPointOnSegment(PointType const &a,
                                                   const PointType &pt0,
                                                   const PointType &pt1) {
  auto const t = findNearestPointOnEdge(a, pt0, pt1);
  if (t < physics::RatioValue(0.)) {
    return physics::ParametricValue(0.);
  } else if (t > physics::RatioValue(1.)) {
    return physics::ParametricValue(1.);
  } else {
    return physics::ParametricValue(static_cast<double>(t));
  }
}

/**
 * @brief Calculate the length of an edge
 * @param[in] edge The input edge to operate on.
 * @return The length of the edge
 */
template <typename PointType>
physics::Distance calculateEdgeLength(std::vector<PointType> const &edge) {
  physics::Distance length(0.);
  for (auto i = 1u; i < edge.size(); ++i) {
    length += distance(edge[i - 1u], edge[i]);
  }
  return length;
}

/**
 * @brief Get the parametric edge points
 * @param[in] edge The input edge to operate on.
 * @return Vector of parametric values on the edge.
 */
template <typename PointType>
std::vector<physics::ParametricValue> getParametricEdgePoints(
    std::vector<PointType> const &edge) {
  std::vector<physics::ParametricValue> resultPoints;
  resultPoints.reserve(edge.size());
  resultPoints.push_back(physics::ParametricValue(0.));
  physics::Distance length(0.);
  for (auto i = 1u; i < edge.size(); ++i) {
    length += distance(edge[i - 1u], edge[i]);
    resultPoints.push_back(
        physics::ParametricValue(static_cast<double>(length)));
  }

  for (auto i = 1u; i < edge.size(); ++i) {
    if (length > physics::Distance(0.)) {
      resultPoints[i] = resultPoints[i] / static_cast<double>(length);
    }
  }
  return resultPoints;
}

/**
 * @brief Calculates parametric point on the edge.
 * @param[in] edge The input edge to operate on.
 * @param[in] t Parameter. 0 will return first point, and 1 last point on the
 * edge.
 * @return Parametric point on the edge.
 */
template <typename PointType>
PointType getParametricPoint(std::vector<PointType> const &edge,
                             physics::Distance const &edgeLength,
                             const physics::ParametricValue &t) {
  if (!edge.empty()) {
    physics::Distance length(0);
    physics::Distance length_t = edgeLength * t;
    for (size_t i = 0; i < edge.size() - 1; i++) {
      const auto &pt0 = edge[i];
      const auto &pt1 = edge[i + 1];
      auto d = distance(pt0, pt1);
      if (d != physics::Distance(0.)) {
        auto length_1 = length + d;
        if (length_1 >= length_t) {
          auto d_t = length_t - length;
          physics::ParametricValue tt(d_t / d);
          return vectorInterpolate(pt0, pt1, tt);
        } else {
          length = length_1;
        }
      }
    }
    return edge.back();
  } else {
    return PointType();
  }
}

/**
 * @brief Calculates parametric point on the edge.
 * @param[in] edge The input edge to operate on.
 * @param[in] t Parameter. 0 will return first point, and 1 last point on the
 * edge.
 * @return Parametric point on the edge.
 */
template <typename PointType>
PointType getParametricPoint(std::vector<PointType> const &edge,
                             const physics::ParametricValue &t) {
  return getParametricPoint(edge, calculateEdgeLength(edge), t);
}

/**
 * @brief Generates sub-edge for given range.
 * @param[in] edge The input edge to operate on.
 * @param[in] trange Specifies parametric range.
 * @return Sub-geometry.
 */
template <typename PointType>
std::vector<PointType> getParametricRange(
    std::vector<PointType> const &edge, physics::Distance const &edgeLength,
    const physics::ParametricRange &trange) {
  typedef std::vector<PointType> EdgeType;
  if (!edge.empty()) {
    EdgeType pts;
    physics::Distance length(0);
    physics::Distance length_t_start = edgeLength * trange.minimum;
    size_t i = 0;
    for (; i < edge.size() - 1 && pts.empty(); i++) {
      const PointType &pt0 = edge[i];
      const PointType &pt1 = edge[i + 1];
      physics::Distance d = distance(pt0, pt1);
      physics::Distance length_1 = length + d;
      if (length_1 >= length_t_start) {
        if (d != physics::Distance(0)) {
          physics::Distance d_t = length_t_start - length;
          physics::ParametricValue tt(d_t / d);
          PointType pt_start = vectorInterpolate(pt0, pt1, tt);
          pts.push_back(pt_start);
        } else {
          pts.push_back(pt1);
        }
        break;
      } else {
        length = length_1;
      }
    }
    if (!pts.empty()) {
      physics::Distance length_t_end = edgeLength * trange.maximum;
      for (; i < edge.size() - 1; i++) {
        const PointType &pt0 = edge[i];
        const PointType &pt1 = edge[i + 1];
        physics::Distance d = distance(pt0, pt1);
        physics::Distance length_1 = length + d;
        if (length_1 >= length_t_end) {
          if (d != physics::Distance(0)) {
            physics::Distance d_t = length_t_end - length;
            physics::ParametricValue tt(d_t / d);
            PointType pt_end = vectorInterpolate(pt0, pt1, tt);
            pts.push_back(pt_end);
          } else {
            pts.push_back(pt1);
          }
          return pts;
        } else {
          pts.push_back(pt1);
          length = length_1;
        }
      }
      pts.push_back(edge.back());
      return pts;
    }
  }
  return EdgeType();
}

/**
 * @brief Generates sub-edge for given range.
 * @param[in] edge The input edge to operate on.
 * @param[in] trange Specifies parametric range.
 * @return Sub-geometry.
 */
template <typename PointType>
std::vector<PointType> getParametricRange(
    std::vector<PointType> const &edge,
    const physics::ParametricRange &trange) {
  return getParametricRange(edge, calculateEdgeLength(edge), trange);
}

/**
 * @brief Finds point on geometry nearest to given point.
 * @param[in] pt Point of interest.
 * @returns Parametric point on geometry nearest to the pt.
 *          Can be invalid (if pt is Invalid(), geometry is empty etc.).
 */
template <typename PointType>
physics::ParametricValue findNearestPointOnEdge(
    std::vector<PointType> const &edge, physics::Distance const &edgeLength,
    const PointType &pt) {
  if (isValid(pt)) {
    if (edge.size() == 0) {
      return physics::ParametricValue();
    } else if ((edge.size() == 1) || (edgeLength == physics::Distance(0.))) {
      return physics::ParametricValue(0);
    } else {
      physics::ParametricValue t_one =
          findNearestPointOnSegment(pt, edge[0], edge[1]);
      PointType pt_nearest = vectorInterpolate(edge[0], edge[1], t_one);
      physics::Distance d_nearest = distance(pt, pt_nearest);
      physics::Distance offset_nearest = distance(pt_nearest, edge[0]);
      physics::Distance running_offset = physics::Distance(0);
      for (size_t i = 1; i + 1 < edge.size(); i++) {
        physics::ParametricValue t =
            findNearestPointOnSegment(pt, edge[i], edge[i + 1]);
        PointType pt_candidate = vectorInterpolate(edge[i], edge[i + 1], t);
        physics::Distance d = distance(pt_candidate, pt);
        running_offset += distance(edge[i - 1], edge[i]);
        if (d < d_nearest) {
          pt_nearest = pt_candidate;
          d_nearest = d;
          offset_nearest = running_offset + distance(pt_nearest, edge[i]);
        }
      }
      return physics::ParametricValue(offset_nearest / edgeLength);
    }
  } else {
    return physics::ParametricValue();
  }
}

/**
 * @brief Find the nearest point on an edge
 * @param[in] edge The input edge to operate on.
 * @param[in] pt Point of interest.
 * @returns Parametric point on geometry nearest to the pt.
 *          Can be invalid (if pt is Invalid(), geometry is empty etc.).
 */
template <typename PointType>
physics::ParametricValue findNearestPointOnEdge(
    std::vector<PointType> const &edge, const PointType &pt) {
  return findNearestPointOnEdge(edge, calculateEdgeLength(edge), pt);
}

/**
 * @brief Calculate the width range and the average width of a pair of edges
 *
 * At maximum 100 supporting points are taken into account to calculate this.
 *
 * @param[in] leftEdge the left-hand border edge as basis for the calculation
 * @param[in] leftEdgeLength the length of the left-hand border edge
 * @param[in] rightEdge the right-hand border edge as basis for the calculation
 * @param[in] rightEdgeLength the length of the left-hand border edge
 *
 * @returns pair containing the range of the width and the average width between
 * the two input edges.
 */
template <typename PointType>
std::pair<physics::MetricRange, physics::Distance> calculateWidthRange(
    std::vector<PointType> const &edgeLeft,
    physics::Distance const &edgeLeftLength,
    std::vector<PointType> const &edgeRight,
    physics::Distance const &edgeRightLength) {
  physics::MetricRange widthRange;
  widthRange.minimum = std::numeric_limits<physics::Distance>::max();
  widthRange.maximum = physics::Distance(0.);
  physics::Distance widthSum(0.);
  size_t widthSumCount(0u);
  physics::ParametricValue parametricStepsize(0.5);
  auto length = (edgeLeftLength + edgeRightLength) * 0.5;
  if (length > physics::Distance(1.)) {
    parametricStepsize =
        std::max(physics::ParametricValue(0.01),
                 physics::ParametricValue(1. / static_cast<double>(length)));
  }
  physics::ParametricValue longitudinalOffset(0.);
  bool endReached = false;
  do {
    auto const pt0 =
        getParametricPoint(edgeLeft, edgeLeftLength, longitudinalOffset);
    auto const pt1 =
        getParametricPoint(edgeRight, edgeRightLength, longitudinalOffset);
    if (!isValid(pt1) || !isValid(pt1)) {
      return std::make_pair(physics::MetricRange(), physics::Distance());
    }
    auto const centerPoint =
        point::vectorInterpolate(pt0, pt1, physics::ParametricValue(0.5));
    if (!isValid(centerPoint)) {
      return std::make_pair(physics::MetricRange(), physics::Distance());
    }
    auto const longTLeft =
        findNearestPointOnEdge(edgeLeft, edgeLeftLength, centerPoint);
    auto const longTRight =
        findNearestPointOnEdge(edgeRight, edgeRightLength, centerPoint);
    if (!isRangeValid(longTLeft) || !isRangeValid(longTRight)) {
      return std::make_pair(physics::MetricRange(), physics::Distance());
    }
    auto const pointOnLeftEdge =
        getParametricPoint(edgeLeft, edgeLeftLength, longTLeft);
    auto const pointOnRightEdge =
        getParametricPoint(edgeRight, edgeRightLength, longTRight);
    if (!isValid(pointOnLeftEdge) || !isValid(pointOnRightEdge)) {
      return std::make_pair(physics::MetricRange(), physics::Distance());
    }
    physics::Distance const width = distance(pointOnLeftEdge, pointOnRightEdge);
    unionRangeWith(widthRange, width);
    widthSum += width;
    widthSumCount++;
    if (longitudinalOffset < physics::ParametricValue(1.)) {
      longitudinalOffset += parametricStepsize;
      if (longitudinalOffset > physics::ParametricValue(1.)) {
        longitudinalOffset = physics::ParametricValue(1.);
      }
    } else {
      endReached = true;
    }
  } while (!endReached);

  auto averageWidth = (widthSum / static_cast<double>(widthSumCount));

  return std::make_pair(widthRange, averageWidth);
}

/**
 * @brief Get an edge between the two given border edges with corresponding
 * lateralAlignment
 *
 * The left and right edges are usually the borders of some road section.
 * This function then calculates a new edge in between two other edges providing
 * e.g. the center edge (lateralAlgignment=0.5) or edge with other lateral
 * shift.
 *
 * @param[in] leftEdge the left-hand border edge as basis for the calculation
 * @param[in] leftEdgeLength the length of the left-hand border edge
 * @param[in] rightEdge the right-hand border edge as basis for the calculation
 * @param[in] rightEdgeLength the length of the left-hand border edge
 * @param[in] lateralAlignment the lateral alignment as TParam [0.;1.] used to
 * calculate the resulting edge. The lateral alignment is relative to the left
 * edge. If lateralAlignment is 1., the left edge is returned, if
 * lateralAlignment is 0., the right edge is returned
 *
 * @throws std::invalid_argument if the lateralAlignment parameter is smaller
 * than 0. or larger than 1.
 */
template <typename PointType>
std::vector<PointType> getLateralAlignmentEdge(
    std::vector<PointType> const &leftEdge,
    physics::Distance const &leftEdgeLength,
    std::vector<PointType> const &rightEdge,
    physics::Distance const &rightEdgeLength,
    physics::ParametricValue const lateralAlignment) {
  typedef std::vector<PointType> EdgeType;

  if (!withinValidInputRange(lateralAlignment)) {
    throw std::invalid_argument(
        "ad::map::point::getLateralAlignmentEdge()"
        " the given lateralAlignment is out of range");
  }

  EdgeType const *primary;
  EdgeType const *secondary;
  physics::Distance primaryLength;
  physics::Distance secondaryLength;
  physics::ParametricValue lateralOffset = lateralAlignment;
  if (leftEdge.size() > rightEdge.size()) {
    primary = &leftEdge;
    primaryLength = leftEdgeLength;
    secondary = &rightEdge;
    secondaryLength = rightEdgeLength;
    lateralOffset = physics::ParametricValue(1.) - lateralAlignment;
  } else {
    primary = &rightEdge;
    primaryLength = rightEdgeLength;
    secondary = &leftEdge;
    secondaryLength = leftEdgeLength;
  }

  std::vector<physics::ParametricValue> const primaryParametric =
      getParametricEdgePoints(*primary);
  EdgeType alignmentEdge;
  alignmentEdge.reserve(primaryParametric.size());
  for (size_t i = 0; i < primaryParametric.size(); i++) {
    PointType const &pri = primary->at(i);
    PointType const sec =
        getParametricPoint(*secondary, secondaryLength, primaryParametric[i]);
    PointType const alignmentPoint = vectorInterpolate(pri, sec, lateralOffset);
    alignmentEdge.push_back(alignmentPoint);
  }
  return alignmentEdge;
}

/**
 * @brief Get an edge between the two given border edges with corresponding
 * lateralAlignment
 *
 * The left and right edges are usually the borders of some road section.
 * This function then calculates a new edge in between two other edges providing
 * e.g. the center edge (lateralAlgignment=0.5) or edge with other lateral
 * shift.
 *
 * Note: if the length of the edges are already know, the overloaded
 * getLateralAlignmentEdge() function can be called.
 *
 * @param[in] leftEdge the left-hand border edge as basis for the calculation
 * @param[in] rightEdge the right-hand border edge as basis for the calculation
 * @param[in] lateralAlignment the lateral alignment as TParam [0.;1.] used to
 * calculate the resulting edge. The lateral alignment is relative to the left
 * edge. If lateralAlignment is 1., the left edge is returned, if
 * lateralAlignment is 0., the right edge is returned
 *
 * @throws std::invalid_argument if the lateralAlignment parameter is smaller
 * than 0. or larger than 1.
 */
template <typename PointType>
std::vector<PointType> getLateralAlignmentEdge(
    std::vector<PointType> const &leftEdge,
    std::vector<PointType> const &rightEdge,
    physics::ParametricValue const lateralAlignment) {
  return getLateralAlignmentEdge(leftEdge, calculateEdgeLength(leftEdge),
                                 rightEdge, calculateEdgeLength(rightEdge),
                                 lateralAlignment);
}

/**
 * @brief get a normalized vector representing the edge direction at edge start
 *
 * If the number of edge points is <= 1, zero is returned.
 * If the two start points of the edge are too close to each other, a third
 * point is used if possible to increase the accuracy.
 */
template <typename PointType>
PointType getEdgeStartDirectionalVector(std::vector<PointType> const edge) {
  if (edge.size() <= 1u) {
    return PointType();
  }

  auto edgeStartVec = edge[1u] - edge[0u];
  // in case the end points are too near to each other we take another point
  // into account if possible
  if ((vectorLength(edgeStartVec) < cEdgePointBorderDistance) &&
      (edge.size() > 2u)) {
    edgeStartVec = edge[2u] - edge[0u];
  }
  return vectorNorm(edgeStartVec);
}

/**
 * @brief get a normalized vector representing the edge direction at edge end
 *
 * If the number of edge points is <= 1, zero is returned.
 * If the two end points of the edge are too close to each other, a third point
 * is used if possible to increase the accuracy.
 */
template <typename PointType>
PointType getEdgeEndDirectionalVector(std::vector<PointType> const edge) {
  if (edge.size() <= 1u) {
    return PointType();
  }

  auto edgeEndVec = edge[edge.size() - 1u] - edge[edge.size() - 2u];
  // in case the end points are too near to each other we take another point
  // into account if possible
  if ((vectorLength(edgeEndVec) < cEdgePointBorderDistance) &&
      (edge.size() > 2u)) {
    edgeEndVec = edge[edge.size() - 1u] - edge[edge.size() - 3u];
  }
  return vectorNorm(edgeEndVec);
}

}  // namespace point
}  // namespace map
}  // namespace ad
