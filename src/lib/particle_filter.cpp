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