#include "util.hpp"
#include <cmath>

using std::vector;

namespace sph {

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

} // end namespace sph