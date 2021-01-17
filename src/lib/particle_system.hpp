#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include "util.hpp"

namespace sph {

struct ParticleSystem {
  ParticleSystem(const std::vector<Vec2d>& pos, const std::vector<Vec2d>& vel,
                 const std::vector<Vec2d>& acc, double mass);
  ParticleSystem(const std::vector<Vec2d>& pos, double mass);
  
  const size_t numberOfParticles;
  const double particleMass;
  std::vector<Vec2d> positions, velocities, accelerations;
  double time;
};

struct Physics {
  virtual ~Physics() {}
  // Adds the effect of velocity-independent forces to the accelerations of particles
  virtual void applyForces(ParticleSystem& ps) const = 0;
  // Adds the effect of damping to the accelerations of particles
  virtual void applyDamping(ParticleSystem& ps) const = 0;
  // Moves particles to resolve collisions
  virtual void resolveCollisions(ParticleSystem& ps) const = 0;
};

class TimeIntegrator {
public:
  virtual ~TimeIntegrator() {};
  void integrate(ParticleSystem& ps, Physics& physics, double duration);

private:
  virtual void step(ParticleSystem& ps, Physics& physics) = 0;
};

class Euler : public TimeIntegrator {
public:
  Euler(double timeStep) : timeStep_{timeStep} {};
  
private:
  void step(ParticleSystem& ps, Physics& physics) override;
  const double timeStep_;
};

class Verlet : public TimeIntegrator {
public:
  Verlet(double timeStep) : timeStep_{timeStep} {};
  
private:
  void step(ParticleSystem& ps, Physics& physics) override;
  const double timeStep_;
};

} // end namespace sph

#endif