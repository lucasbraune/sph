#ifndef LOOP_STRATEGY_HPP
#define LOOP_STRATEGY_HPP

#include <cassert>
#include <type_traits>
#include "particle_system.hpp"

namespace sph {

class TrivialLoopStrategy final {
public:
  void syncWith(const ParticleSystem& ps) { ps_ = &ps; };
  
  template<class SummandFn>
  std::result_of_t<SummandFn(const Particle&)>
  accumulate(SummandFn&& summand, const ParticleSystem& ps, const Disk&) const
  {
    assert(inSync(ps));
    auto sum = std::result_of_t<SummandFn(const Particle&)>{};
    for (auto& neighbor : ps_->particles) {
      sum += summand(neighbor);
    }
    return sum;
  }

private:
  bool inSync(const ParticleSystem& ps) const
  {
    return ps_ == &ps && ps_->time == ps.time;
  }
  
  const ParticleSystem* ps_;
};

namespace detail {

struct SubdividedInterval {
  SubdividedInterval(double left, double right, size_t subdivisions);
  size_t subdivision(double x) const;

  const double left;
  const double right;
  const size_t subdivisions;
};

} // namespace detail

class GridBasedLoopStrategy final {
public:
  GridBasedLoopStrategy(const Rectangle& rect, size_t rows, size_t cols);
  void syncWith(const ParticleSystem& ps);
  
  template<class SummandFn>
  std::result_of_t<SummandFn(const Particle&)>
  accumulate(SummandFn&& summand, const ParticleSystem& ps, const Disk& neighborhood) const
  {
    assert(inSync(ps));
    const auto minI = row(neighborhood.center[0] - neighborhood.radius);
    const auto maxI = row(neighborhood.center[0] + neighborhood.radius);
    const auto minJ = column(neighborhood.center[1] - neighborhood.radius);
    const auto maxJ = column(neighborhood.center[1] + neighborhood.radius);
    auto sum = std::result_of_t<SummandFn(const Particle&)>{};
    for (size_t i=minI; i<=maxI; ++i) {
      for (size_t j=minJ; j<=maxJ; ++j) {
        for (auto* particlePtr : grid_(i, j)) {
          sum += summand(*particlePtr);
        }
      }
    }
    return sum;
  }

private:
  size_t row(double x) const { return width_.subdivision(x); }
  size_t column(double y) const { return height_.subdivision(y); }
  bool inSync(const ParticleSystem& ps) const
  {
    return ps_ == &ps && ps_->time == ps.time;
  }
  
  Matrix<std::vector<const Particle*>> grid_;
  const detail::SubdividedInterval width_, height_;
  const ParticleSystem* ps_;
};

GridBasedLoopStrategy makeGridBasedLoopStrategy(const Rectangle& rect, double minimumCellLength);

} // namespace sph

#endif