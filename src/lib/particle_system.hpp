#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include "util.hpp"

namespace sph {

struct ParticleSystem {
  ParticleSystem(const std::vector<Vec2d>& initialPositions,
                 const std::vector<Vec2d>& initialVelocities,
                 const std::vector<Vec2d>& initialAccelerations,
                 double particleMass);
  ParticleSystem(const std::vector<Vec2d>& initialPositions, double particleMass);
  
  // Properties
  const size_t numberOfParticles;
  const double particleMass;

  // State
  std::vector<Vec2d> positions, velocities, accelerations;
  double time;
};

struct Physics {
  virtual ~Physics() {}
  virtual void applyForces(ParticleSystem& ps) const = 0;
  virtual void applyDamping(ParticleSystem& ps) const = 0;
  virtual void resolveCollisions(ParticleSystem& ps) const = 0;
};

class TimeIntegrator {
public:
  virtual ~TimeIntegrator() {};
  void integrate(ParticleSystem& ps, Physics& physics, double duration);

private:
  virtual void step(ParticleSystem& ps, Physics& physics) = 0;
};

class EulerIntegrator : public TimeIntegrator {
public:
  EulerIntegrator(double timeStep) : timeStep_{timeStep} {};
  
private:
  void step(ParticleSystem& ps, Physics& physics) override;
  const double timeStep_;
};

class VerletIntegrator : public TimeIntegrator {
public:
  VerletIntegrator(double timeStep) : timeStep_{timeStep} {};
  
private:
  void step(ParticleSystem& ps, Physics& physics) override;
  const double timeStep_;
};

} // end namespace sph

#endif