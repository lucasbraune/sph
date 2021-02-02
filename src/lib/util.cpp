#include "util.hpp"
#include <cmath> // hypot
#include <random>

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

double sph::norm(const Vec2d& x)
{
  return std::hypot(x[0], x[1]);
}

Vec2d sph::randomVec2d(const Rectangle& rect)
{
  static auto dist = std::uniform_real_distribution<double>{};
  static auto re = std::default_random_engine{};
  constexpr auto affineFn = [](double a, double b, double x) {
    return a + (b - a) * x;
  };
  return {affineFn(rect.xmin, rect.xmax, dist(re)),
          affineFn(rect.ymin, rect.ymax, dist(re))};
}