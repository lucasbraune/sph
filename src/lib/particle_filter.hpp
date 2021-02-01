#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <vector>
#include "particle_system.hpp"
#include "util.hpp"
#include "range/v3/view/any_view.hpp"
#include "range/v3/view/iota.hpp"

namespace sph {

struct ParticleFilter {
  virtual ~ParticleFilter() {};
  virtual std::unique_ptr<ParticleFilter> clone() const = 0;
  virtual void syncWith(const ParticleSystem& ps) = 0;
  virtual ranges::any_view<const Particle&> particlesIn(const Disk& neighborhood) const = 0; 
};

class TrivialFilter final : public ParticleFilter {
public:
  void syncWith(const ParticleSystem& ps) { ps_ = &ps; };
  std::unique_ptr<ParticleFilter> clone() const;
  ranges::any_view<const Particle&> particlesIn(const Disk&) const final;

private:
  const ParticleSystem* ps_;
};

} // namespace sph

#endif