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

Vec2d operator+(const Vec2d& a, const Vec2d& b);
Vec2d operator-(const Vec2d& a, const Vec2d& b);
Vec2d operator*(const double lambda, const Vec2d& a);
void operator+=(Vec2d& a, const Vec2d& b);
void operator-=(Vec2d& a, const Vec2d& b);
void operator*=(Vec2d& a, const double lambda);

#endif