#include "util.hpp"

vector<Vec2d> randomPositions(const Rectangle region, const size_t N) {
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

Vec2d operator+(const Vec2d& a, const Vec2d& b) {
  Vec2d res;
  for (size_t i=0; i < a.size(); i++) {
    res[i] = a[i] + b[i];
  }
  return res;
}

Vec2d operator-(const Vec2d& a, const Vec2d& b) {
  Vec2d res;
  for (size_t i=0; i < a.size(); i++) {
    res[i] = a[i] - b[i];
  }
  return res;
}

Vec2d operator*(const double lambda, const Vec2d& a) {
  Vec2d res;
  for (size_t i=0; i< a.size(); i++) {
    res[i] = lambda * a[i];
  }
  return res;
}

void operator+=(Vec2d& a, const Vec2d& b) {
  for (size_t i=0; i < a.size(); i++) {
    a[i] += b[i];
  }
}

void operator-=(Vec2d& a, const Vec2d& b) {
  for (size_t i=0; i < a.size(); i++) {
    a[i] -= b[i];
  }
}

void operator*=(Vec2d& a, const double lambda) {
  for (size_t i=0; i < a.size(); i++) {
    a[i] *= lambda;
  }
}