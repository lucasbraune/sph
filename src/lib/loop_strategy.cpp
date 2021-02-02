#include "loop_strategy.hpp"
#include <cassert>

using namespace sph;

sph::detail::SubdividedInterval::SubdividedInterval(double left, double right,
                                                    size_t subdivisions) : 
  left{left}, right{right}, subdivisions{subdivisions}
{
  assert(left < right && subdivisions > 0);
}

size_t sph::detail::SubdividedInterval::subdivision(double x) const
{
  if (x < left) return 0;
  auto i = static_cast<size_t>(subdivisions * (x - left) / (right - left));
  return i < subdivisions ? i : subdivisions - 1;
}

sph::GridBasedLoopStrategy::GridBasedLoopStrategy(const Rectangle& rect, size_t rows, size_t cols) :
  grid_{rows, cols},
  width_{rect.xmin, rect.xmax, rows},
  height_{rect.ymin, rect.ymax, cols}
{}

namespace {

template<class Container>
static void clearEntries(Matrix<Container>& matrix)
{
  for (size_t i=0; i<matrix.rows(); ++i) {
    for (size_t j=0; j<matrix.columns(); ++j) {
      matrix(i, j).clear();
    }
  }
}

} // namespace

void sph::GridBasedLoopStrategy::syncWith(const ParticleSystem& ps) {
  ps_ = &ps;
  clearEntries(grid_);
  for (auto& particle : ps.particles) {
    const auto i = row(particle.pos[0]);
    const auto j = column(particle.pos[1]);
    grid_(i, j).emplace_back(&particle);
  }
};

GridBasedLoopStrategy
sph::makeGridBasedLoopStrategy(const Rectangle& rect, double minimumCellLength)
{
  assert(minimumCellLength > 0);
  auto rows = static_cast<size_t>(width(rect) / minimumCellLength);
  auto columns = static_cast<size_t>(height(rect) / minimumCellLength);
  return {rect, rows, columns};
}