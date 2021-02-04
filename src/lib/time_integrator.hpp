#ifndef TIME_INTEGRATOR_HPP
#define TIME_INTEGRATOR_HPP

#include "particle_system.hpp"

namespace sph {

class TimeIntegrator {
public:
  virtual ~TimeIntegrator() {};
  void integrate(ParticleSystem& ps, Physics& physics, double duration);

private:
  virtual void step(ParticleSystem& ps, Physics& physics) = 0;
};

class Euler final : public TimeIntegrator {
public:
  Euler(double timeStep) : timeStep_{timeStep} {};
  
private:
  void step(ParticleSystem& ps, Physics& physics) override;
  const double timeStep_;
};

class Verlet final : public TimeIntegrator {
public:
  Verlet(double timeStep) : timeStep_{timeStep} {};
  
private:
  void step(ParticleSystem& ps, Physics& physics) override;
  const double timeStep_;
};

} // namespace sph

#endif