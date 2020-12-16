#include "util.hpp"
#include <random>
#include <cmath>

using std::vector;

namespace sph {

Vec2d& operator+=(Vec2d& a, const Vec2d& b)
{
  a[0] += b[0];
  a[1] += b[1];
  return a;
}

Vec2d& operator-=(Vec2d& a, const Vec2d& b)
{
  a[0] -= b[0];
  a[1] -= b[1];
  return a;
}

Vec2d& operator*=(Vec2d& a, const double lambda)
{
  a[0] *= lambda;
  a[1] *= lambda;
  return a;
}

Vec2d& operator/=(Vec2d& a, const double lambda)
{
  return a *= 1.0 / lambda;
}

Vec2d operator+(Vec2d a, const Vec2d& b)
{
  return a += b;
}

Vec2d operator-(Vec2d a, const Vec2d& b)
{
  return a -= b;
}

Vec2d operator*(const double lambda, Vec2d a)
{
  return a *= lambda;
}

Vec2d operator*(Vec2d a, const double lambda)
{
  return a *= lambda;
}

Vec2d operator/(Vec2d a, const double lambda)
{
  return a /= lambda;
}

double operator*(const Vec2d& a, const Vec2d& b)
{
  return a[0] * b[0] + a[1] * b[1];
}

double norm(const Vec2d& x)
{
  return hypot(x[0], x[1]);
}

double dist(const Vec2d& x, const Vec2d& y)
{
  return norm(x - y);
}

Vec2d unit(const Vec2d& x)
{
  return x / norm(x);
}

Vec2d project(const Vec2d& x, const Vec2d& unitNormal)
{
  return (x * unitNormal) * unitNormal;
}

double Rectangle::width() const
{
  return xmax - xmin;
}

double Rectangle::height() const
{
  return ymax - ymin;
}

Vec2d Rectangle::bottomLeft() const
{
  return Vec2d{xmin, ymin};
}

vector<Vec2d> randomVectors(const Rectangle region, const size_t N) {
  vector<Vec2d> result;
  std::uniform_real_distribution<double> xDist{region.xmin, region.xmax};
  std::uniform_real_distribution<double> yDist{region.ymin, region.ymax};
  std::default_random_engine re;
  for (size_t i=0; i<N; i++) {
    Vec2d randomVector{xDist(re), yDist(re)};
    result.push_back(randomVector);
  }
  return result;
}

} // end namespace sph