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

struct Force {
public:
  virtual ~Force() {}
  virtual void apply(double time, double particleMass, const std::vector<Vec2d>& positions,
                     std::vector<Vec2d>& accelerations) const = 0;
};

struct Damping {
  virtual ~Damping() {}
  virtual Vec2d acceleration(double time, double mass, const Vec2d& velocity) const = 0;
  static Vec2d acceleration(const std::vector<const Damping*>& dampings, double time, double mass,
                            const Vec2d& velocity);
};

struct Collidable {
  virtual ~Collidable() {}
  virtual void resolveCollisions(std::vector<Vec2d>& positions, std::vector<Vec2d>& velocities,
                                 double time) const = 0;
};

struct Physics {
  virtual ~Physics() {}
  virtual const std::vector<const Force*>& forcePtrs() const = 0;
  virtual const std::vector<const Damping*>& dampingPtrs() const = 0;
  virtual const std::vector<const Collidable*>& collidablePtrs() const = 0;
};

struct TimeIntegrator {
  virtual ~TimeIntegrator() {};
  virtual void step(ParticleSystem& ps, Physics& physics) = 0;
  void integrate(ParticleSystem& ps, Physics& physics, double duration);
};

class EulerIntegrator : public TimeIntegrator {
public:
  EulerIntegrator(double timeStep) : timeStep_{timeStep} {};
  void step(ParticleSystem& ps, Physics& physics) override;

private:
  const double timeStep_;
};

class VerletIntegrator : public TimeIntegrator {
public:
  VerletIntegrator(double timeStep) : timeStep_{timeStep} {};
  void step(ParticleSystem& ps, Physics& physics) override;

private:
  const double timeStep_;
  std::vector<Vec2d> nextForceAcc_;
};

} // end namespace sph

#endif