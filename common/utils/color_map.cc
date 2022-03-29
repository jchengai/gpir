/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "common/utils/color_map.h"

#include <unordered_map>

namespace common {

static std::unordered_map<Color, ColorRGBA> color_map{
    {kBlack, ColorRGBA(0.0, 0.0, 0.0, 1.0)},
    {kWhite, ColorRGBA(1.0, 1.0, 1.0, 1.0)},
    {kRed, ColorRGBA(1.0, 0.0, 0.0, 1.0)},
    {kPink, ColorRGBA(1.0, 0.44, 0.70, 1.0)},
    {kGreen, ColorRGBA(0.0, 1.0, 0.0, 1.0)},
    {kBlue, ColorRGBA(0.0, 0.0, 1.0, 1.0)},
    {kMarine, ColorRGBA(0.5, 1.0, 0.83, 1.0)},
    {kYellow, ColorRGBA(1.0, 1.0, 0.0, 1.0)},
    {kCyan, ColorRGBA(0.0, 1.0, 1.0, 1.0)},
    {kMagenta, ColorRGBA(1.0, 0.0, 1.0, 1.0)},
    {kViolet, ColorRGBA(0.93, 0.43, 0.93, 1.0)},
    {kOrangeRed, ColorRGBA(1.0, 0.275, 0.0, 1.0)},
    {kOrange, ColorRGBA(1.0, 0.65, 0.0, 1.0)},
    {kDarkOrange, ColorRGBA(1.0, 0.6, 0.0, 1.0)},
    {kGold, ColorRGBA(1.0, 0.84, 0.0, 1.0)},
    {kGreenYellow, ColorRGBA(0.5, 1.0, 0.0, 1.0)},
    {kFroestGreen, ColorRGBA(0.13, 0.545, 0.13, 1.0)},
    {kSpringGreen, ColorRGBA(0.0, 1.0, 0.5, 1.0)},
    {kSkyBlue, ColorRGBA(0.0, 0.749, 1.0, 1.0)},
    {kMediumOrchid, ColorRGBA(0.729, 0.333, 0.827, 1.0)},
    {kGrey, ColorRGBA(0.5, 0.5, 0.5, 1.0)},
    {kRoyalBlue, ColorRGBA(0.25, 0.41, 0.88, 1.0)}};

ColorRGBA ColorMap::at(const Color color, const double a) {
  auto color_rgba = color_map.at(color);
  color_rgba.a = a;
  return color_rgba;
}

}  // namespace common
