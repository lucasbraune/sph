#include "particle_filter.hpp"
#include "range/v3/view/cartesian_product.hpp"
#include "range/v3/view/transform.hpp"
#include "range/v3/view/join.hpp"
#include <cassert>

using namespace sph;

ranges::any_view<const Particle&> sph::TrivialFilter::particlesIn(const Disk&) const
{
  assert(ps_);
  return ps_->particles | ranges::views::all;
}

std::unique_ptr<ParticleFilter> sph::TrivialFilter::clone() const
{
  return std::make_unique<TrivialFilter>(*this);
}

std::unique_ptr<ParticleFilter> sph::GridFilter::clone() const
{
  return std::make_unique<GridFilter>(*this);
}

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

ranges::closed_iota_view<size_t, size_t>
sph::detail::SubdividedInterval::subdivisionRange(double xmin, double xmax) const
{
  return ranges::views::closed_iota(subdivision(xmin), subdivision(xmax));
}

sph::GridFilter::GridFilter(const Rectangle& rect, size_t rows, size_t cols) : 
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

void sph::GridFilter::syncWith(const ParticleSystem& ps)
{ 
  clearEntries(grid_);
  for (auto& particle : ps.particles) {
    auto i = row(particle.pos);
    auto j = column(particle.pos);
    grid_(i, j).emplace_back(&particle);
  }
};

ranges::any_view<const Particle&> sph::GridFilter::particlesIn(const Disk& neighborhood) const
{
  auto accessGrid = [&grid = grid_](auto&& indexPair)->auto& {
    return grid(std::get<0>(indexPair), std::get<1>(indexPair));
  };
  constexpr auto deref = [](auto ptr)->auto&{ return *ptr; };
  using namespace ranges;
  return views::cartesian_product(rowRange(neighborhood), columnRange(neighborhood)) |
         views::transform(accessGrid) |
         views::join |
         views::transform(deref);
}

ranges::closed_iota_view<size_t, size_t>
sph::GridFilter::rowRange(const Disk& neighborhood) const
{
  return width_.subdivisionRange(neighborhood.center[0] - neighborhood.radius,
                                  neighborhood.center[0] + neighborhood.radius);
}

ranges::closed_iota_view<size_t, size_t>
sph::GridFilter::columnRange(const Disk& neighborhood) const
{
  return height_.subdivisionRange(neighborhood.center[1] - neighborhood.radius,
                                  neighborhood.center[1] + neighborhood.radius);
}

GridFilter sph::makeGridFilter(const Rectangle& rect, double minimumCellLength) {
  assert(minimumCellLength > 0);
  auto rows = static_cast<size_t>(width(rect) / minimumCellLength);
  auto columns = static_cast<size_t>(height(rect) / minimumCellLength);
  return GridFilter{rect, rows, columns};
};