#ifndef UTIL_HPP
#define UTIL_HPP

#include <array>
#include <vector>
#include <random>

using Vec2d = std::array<double, 2>;
using std::vector;

struct Rectangle {
  const double xmin, ymin, xmax, ymax;
};

vector<Vec2d> randomPositions(const Rectangle region, const size_t N);

#endif