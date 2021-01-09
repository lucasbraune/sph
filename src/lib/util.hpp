#ifndef UTIL_HPP
#define UTIL_HPP

#include <array>
#include <vector>
#include <cmath>

namespace sph {

using Vec2d = std::array<double, 2>;

constexpr Vec2d ZERO_VECTOR{};

inline Vec2d& operator+=(Vec2d& a, const Vec2d& b)
{
  a[0] += b[0];
  a[1] += b[1];
  return a;
}
inline Vec2d& operator-=(Vec2d& a, const Vec2d& b)
{
  a[0] -= b[0];
  a[1] -= b[1];
  return a;
}
inline Vec2d& operator*=(Vec2d& a, const double lambda)
{
  a[0] *= lambda;
  a[1] *= lambda;
  return a;
}
inline Vec2d& operator/=(Vec2d& a, double lambda) { return a *= 1.0 / lambda; }
inline Vec2d operator+(Vec2d a, const Vec2d& b) { return a += b; }
inline Vec2d operator-(Vec2d a, const Vec2d& b) { return a -= b; }
inline Vec2d operator*(double lambda, Vec2d a) { return a *= lambda; }
inline Vec2d operator*(Vec2d a, double lambda) { return a *= lambda; }
inline Vec2d operator/(Vec2d a, double lambda) { return a /= lambda; }

inline double operator*(const Vec2d& a, const Vec2d& b) { return a[0] * b[0] + a[1] * b[1]; }
inline Vec2d project(const Vec2d& x, const Vec2d& unitNormal)
{
  return (x * unitNormal) * unitNormal;
}
inline double norm(const Vec2d& x) { return hypot(x[0], x[1]); }
inline double dist(const Vec2d& x, const Vec2d& y) { return norm(x - y); }
inline Vec2d unit(const Vec2d& x) { return x / norm(x); }

struct Rectangle {
  double width() const { return xmax - xmin; }
  double height() const { return ymax - ymin; }
  Vec2d bottomLeft() const { return Vec2d{xmin, ymin}; }
  const double xmin = 0.0, ymin = 0.0, xmax = 1.0, ymax = 1.0;
};

} // end namespace sph



#endif