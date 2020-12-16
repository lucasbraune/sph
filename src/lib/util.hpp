#ifndef UTIL_HPP
#define UTIL_HPP

#include <array>
#include <vector>

namespace sph {

using Vec2d = std::array<double, 2>;

constexpr Vec2d ZERO_VECTOR{0.0, 0.0};

Vec2d& operator+=(Vec2d& a, const Vec2d& b);
Vec2d& operator-=(Vec2d& a, const Vec2d& b);
Vec2d& operator*=(Vec2d& a, double lambda);
Vec2d& operator/=(Vec2d& a, double lambda);

Vec2d operator+(Vec2d a, const Vec2d& b);
Vec2d operator-(Vec2d a, const Vec2d& b);
Vec2d operator*(double lambda, Vec2d a);
Vec2d operator*(Vec2d a, double lambda);
Vec2d operator/(Vec2d a, double lambda);

double operator*(const Vec2d& a, const Vec2d& b);
double norm(const Vec2d& x);
double dist(const Vec2d& x, const Vec2d& y);
Vec2d unit(const Vec2d& x);
Vec2d project(const Vec2d& x, const Vec2d& unitNormal);

struct Rectangle {
  double width() const;
  double height() const;
  Vec2d bottomLeft() const;
  const double xmin, ymin, xmax, ymax;
};

} // end namespace sph



#endif