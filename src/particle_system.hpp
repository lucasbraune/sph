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
                 vector<Force*> forces, Damping& damping, const double particleMass,
                 const double timeStep);
  const vector<Vec2d>& positions() const;
  double time() const;
  void integrate(double time);
  
private:
  void step(const double dt);
  
  const size_t numberOfParticles_;
  vector<Vec2d> positions_, velocities_, accelerations_;
  vector<Force*> forces_; 
  Damping& damping_; 
  const double particleMass_;
  double time_;

  // Used in time steps
  const double timeStep_;
  vector<Vec2d> nextForceAcc_;
};

#endif