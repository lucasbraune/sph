#ifndef UTIL_HPP
#define UTIL_HPP

#include <array>
#include <vector>
#include <random>

using Vec2d = std::array<double, 2>;
using std::vector;

constexpr Vec2d ZERO_VECTOR{0.0, 0.0};

Vec2d& operator+=(Vec2d& a, const Vec2d& b);
Vec2d& operator-=(Vec2d& a, const Vec2d& b);
Vec2d& operator*=(Vec2d& a, double lambda);
Vec2d operator+(Vec2d a, const Vec2d& b);
Vec2d operator-(Vec2d a, const Vec2d& b);
Vec2d operator*(double lambda, Vec2d a);
Vec2d operator*(Vec2d a, double lambda);

double norm(const Vec2d& x);
double dist(const Vec2d& x, const Vec2d& y);

struct Rectangle {
  double width() const;
  double height() const;
  Vec2d bottomLeft() const;
  Vec2d upperRight() const;
  
  const double xmin, ymin, xmax, ymax;
};

vector<Vec2d> randomVectors(const Rectangle region, const size_t numberOfVectors);

#endif