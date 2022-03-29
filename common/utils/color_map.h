/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <std_msgs/ColorRGBA.h>

#include <string>

namespace common {

enum Color {
  kBlack = 0,
  kWhite = 1,
  kRed = 2,
  kPink = 3,
  kGreen = 4,
  kBlue = 5,
  kMarine = 6,
  kYellow = 7,
  kCyan = 8,
  kMagenta = 9,
  kOrangeRed = 10,
  kOrange = 11,
  kDarkOrange = 12,
  kGold = 13,
  kGreenYellow = 14,
  kFroestGreen = 15,
  kSpringGreen = 16,
  kSkyBlue = 17,
  kMediumOrchid = 18,
  kGrey = 19,
  kViolet = 20,
  kRoyalBlue = 21,
};

struct ColorRGBA {
  double r = 1.0;
  double g = 0.0;
  double b = 0.0;
  double a = 1.0;

  ColorRGBA(const double R, const double G, const double B, const double A)
      : r(R), g(G), b(B), a(A) {}

  std_msgs::ColorRGBA toRosMsg() const {
    std_msgs::ColorRGBA color_rgba;
    color_rgba.r = r;
    color_rgba.g = g;
    color_rgba.b = b;
    color_rgba.a = a;
    return color_rgba;
  }
};

class ColorMap {
 public:
  static ColorRGBA at(const Color color, const double a = 1.0);
};
}  // namespace common
