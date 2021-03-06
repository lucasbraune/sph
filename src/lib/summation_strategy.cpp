#include "summation_strategy.hpp"
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

namespace {

size_t rows(const Rectangle& rect, double minimumCellLength)
{
  assert(minimumCellLength > 0);
  return static_cast<size_t>(width(rect) / minimumCellLength);
}

size_t columns(const Rectangle& rect, double minimumCellLength)
{
  assert(minimumCellLength > 0);
  return static_cast<size_t>(height(rect) / minimumCellLength);
}

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

sph::GridBasedSummation::GridBasedSummation(const Rectangle& rect, size_t rows, size_t cols) :
  grid_{rows, cols},
  width_{rect.xmin, rect.xmax, rows},
  height_{rect.ymin, rect.ymax, cols}
{}

sph::GridBasedSummation::GridBasedSummation(const Rectangle& rect, double minimumCellLength) :
  GridBasedSummation{rect, rows(rect, minimumCellLength), columns(rect, minimumCellLength)}
{}

void sph::GridBasedSummation::syncWith(const ParticleSystem& ps) {
  ps_ = &ps;
  lastSynchronized_ = ps.time;
  clearEntries(grid_);
  for (auto& particle : ps.particles) {
    const auto i = row(particle.pos[0]);
    const auto j = column(particle.pos[1]);
    grid_(i, j).emplace_back(&particle);
  }
};