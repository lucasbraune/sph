#ifndef UTIL_HPP
#define UTIL_HPP

#include <array>
#include <vector>

namespace sph {

using Vec2d = std::array<double, 2>;

constexpr Vec2d ZERO_VECTOR{0.0, 0.0};

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

double norm(const Vec2d& x);
double dist(const Vec2d& x, const Vec2d& y);
Vec2d unit(const Vec2d& x);

struct Rectangle {
  double width() const { return xmax - xmin; }
  double height() const { return ymax - ymin; }
  Vec2d bottomLeft() const { return Vec2d{xmin, ymin}; }
  const double xmin, ymin, xmax, ymax;
};

} // end namespace sph



#endif