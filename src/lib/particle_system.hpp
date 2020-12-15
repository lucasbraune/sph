#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include <memory>
#include <cmath>
#include "util.hpp"

using std::vector;

struct ParticleSystem {
  ParticleSystem(const vector<Vec2d>& initialPositions,
                 const vector<Vec2d>& initialVelocities,
                 double particleMass);
  ParticleSystem(size_t numberOfParticles, double totalMass, Rectangle region);
  
  // Properties
  const size_t numberOfParticles;
  const double particleMass;

  // State
  vector<Vec2d> positions, velocities, accelerations;
  double time;
};

struct Force {
public:
  virtual ~Force() {}
  virtual void apply(double time, double particleMass, const vector<Vec2d>& positions,
                     vector<Vec2d>& accelerations) const = 0;
};

struct Damping {
  virtual ~Damping() {}
  virtual Vec2d acceleration(double time, double mass, const Vec2d& velocity) const = 0;
  static Vec2d acceleration(const vector<const Damping*>& dampings, double time, double mass,
                            const Vec2d& velocity);
};

struct Collidable {
  virtual ~Collidable() {}
  virtual void resolveCollisions(vector<Vec2d>& positions, vector<Vec2d>& velocities,
                                 double time) const = 0;
};

struct Physics {
  virtual ~Physics() {}
  virtual const vector<const Force*>& forcePtrs() const = 0;
  virtual const vector<const Damping*>& dampingPtrs() const = 0;
  virtual const vector<const Collidable*>& collidablePtrs() const = 0;
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
  static inline Vec2d nextVelocity(Vec2d currVel, Vec2d currAcc, Vec2d nextForceAcc,
                                   const vector<const Damping*>& dampings, double mass, double time,
                                   double timeStep);
  const double timeStep_;
  vector<Vec2d> nextForceAcc_;
};

#endif