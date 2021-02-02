#ifndef LOOP_STRATEGY_HPP
#define LOOP_STRATEGY_HPP

#include <vector>
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
    return &ps == ps_ && ps.time == ps_->time;
  }
  
  const ParticleSystem* ps_;
};

} // namespace sph

#endif