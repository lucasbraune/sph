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

class Force {
public:
  virtual ~Force() {}
  // Adds this force's contribution to the acceleration of the ith particle to the ith entry
  // of the input vector.
  void apply(const ParticleSystem& ps, std::vector<Vec2d>& accelerations) const;
  void apply(ParticleSystem& ps) const;

private:
  virtual void apply(const std::vector<Vec2d>& positions, double particleMass, double time,
                     std::vector<Vec2d>& accelerations) const = 0;
};

struct Damping {
  virtual ~Damping() {}
  void apply(const ParticleSystem& ps, std::vector<Vec2d>& accelerations) const;
  void apply(ParticleSystem& ps) const;
  virtual Vec2d acceleration(const Vec2d& velocity, double mass) const = 0;
};

class Collidable {
public:
  virtual ~Collidable() {}
  void resolveCollisions(ParticleSystem& ps) const;

private:
  virtual void resolveCollisions(std::vector<Vec2d>& positions, std::vector<Vec2d>& velocities,
                                 double time) const = 0;
};

struct Physics {
  virtual ~Physics() {}
  // The pointers returned by the next two functions are guaranteed to be not null
  virtual const std::vector<const Force*>& forcePtrs() const = 0;
  virtual const std::vector<const Collidable*>& collidablePtrs() const = 0;
  virtual const std::vector<const Damping*>& dampingPtrs() const = 0;

  // Warning: may return nullptr
  const Damping* dampingPtr() const
  {
    return dampingPtrs().size() > 0 ? dampingPtrs()[0] : nullptr;
  }
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
};

} // end namespace sph

#endif