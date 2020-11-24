#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include <memory>
#include <cmath>
#include "util.hpp"

using std::vector;

class Force {
public:
  virtual ~Force() {}
  virtual void apply(const double time, const double particleMass, const vector<Vec2d>& positions,
                     vector<Vec2d>& accelerations) const = 0;
};

class Damping {
public:
  virtual ~Damping() {}
  virtual Vec2d acceleration(const double time, const double mass, const Vec2d velocity) const = 0;
};

class ParticleSystem {
public:
  ParticleSystem(const vector<Vec2d>& initialPositions, const vector<Vec2d>& initialVelocities,
                 vector<Force*> forces, Damping& damping, const double particleMass);

  const vector<Vec2d>& positions() const;
  double time() const;
  
  const size_t numberOfParticles_;
  const double particleMass_;
  vector<Vec2d> positions_, velocities_, accelerations_;
  vector<Force*> forces_; 
  Damping& damping_; 
  double time_;
};

class TimeIntegrator {
public:
  virtual void step(ParticleSystem& ps) = 0;
  void integrate(ParticleSystem& ps, double duration);
};

class EulerIntegrator : public TimeIntegrator {
public:
  EulerIntegrator(double timeStep = 0.01);
  void step(ParticleSystem& ps) override;

private:
  const double timeStep_;
};

class VerletIntegrator : public TimeIntegrator {
public:
  VerletIntegrator(double timeStep = 0.01);
  void step(ParticleSystem& ps) override;

private:
  static inline Vec2d nextVelocity(Vec2d currVel, Vec2d currAcc, Vec2d nextForceAcc,
                                   const Damping& damping, double mass, double time,
                                   double timeStep);
  const double timeStep_;
  vector<Vec2d> nextForceAcc_;
};

#endif