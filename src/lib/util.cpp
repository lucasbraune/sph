#include "util.hpp"

using namespace sph;

sph::Rectangle::Rectangle(double xmin, double ymin, double xmax, double ymax) :
  xmin{xmin}, ymin{ymin}, xmax{xmax}, ymax{ymax}
{
  assert(xmax > xmin && ymax > ymin);
}

sph::Disk::Disk(const Vec2d& center, double radius) :
  center{center}, radius{radius}
{
  assert(radius > 0);
}

Vec2d sph::project(const Vec2d& x, const Vec2d& normal)
{
  return (dotProduct(x, normal) / dotProduct(normal, normal)) * normal;
}
