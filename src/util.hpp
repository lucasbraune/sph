#ifndef UTIL_HPP
#define UTIL_HPP

#include <array>
#include <vector>
#include <random>

using Vec2d = std::array<double, 2>;
using std::vector;

constexpr Vec2d ZERO_VECTOR{0.0, 0.0};

Vec2d operator+(const Vec2d& a, const Vec2d& b);
Vec2d operator-(const Vec2d& a, const Vec2d& b);
Vec2d operator*(const double lambda, const Vec2d& a);
Vec2d operator*(const Vec2d& a, const double lambda);
void operator+=(Vec2d& a, const Vec2d& b);
void operator-=(Vec2d& a, const Vec2d& b);
void operator*=(Vec2d& a, const double lambda);
double norm(Vec2d x);
double dist(Vec2d x, Vec2d y);

struct Rectangle {
  const double xmin, ymin, xmax, ymax;
};

vector<Vec2d> randomVectors(const Rectangle region, const size_t numberOfVectors);

#endif