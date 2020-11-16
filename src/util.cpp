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