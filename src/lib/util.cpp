#include "util.hpp"
#include <cmath>

Vec2d operator+(const Vec2d& a, const Vec2d& b)
{
  return Vec2d{a[0] + b[0], a[1] + b[1]};
}

Vec2d operator-(const Vec2d& a, const Vec2d& b)
{
  return Vec2d{a[0] - b[0], a[1] - b[1]};
}

Vec2d operator*(const double lambda, const Vec2d& a)
{
  return Vec2d{lambda * a[0], lambda * a[1]};
}

Vec2d operator*(const Vec2d& a, const double lambda)
{
  return lambda * a;
}

void operator+=(Vec2d& a, const Vec2d& b)
{
  a[0] += b[0];
  a[1] += b[1];
}

void operator-=(Vec2d& a, const Vec2d& b)
{
  a[0] -= b[0];
  a[1] -= b[1];
}

void operator*=(Vec2d& a, const double lambda)
{
  a[0] *= lambda;
  a[1] *= lambda;
}

double norm(Vec2d x)
{
  return hypot(x[0], x[1]);
}

double dist(Vec2d x, Vec2d y)
{
  return norm(x - y);
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