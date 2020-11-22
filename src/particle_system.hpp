#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include "util.hpp"
#include <functional> 
#include <memory>
#include <cmath>

using std::vector;
using std::function;

class Force {
public:
  virtual ~Force() {}
  virtual void apply(const double time, const double particleMass, const vector<Vec2d>& positions,
                     vector<Vec2d>& accelerations) = 0;
};

class Damping {
public:
  virtual ~Damping() {}
  virtual void apply(const double time, const double particleMass, const vector<Vec2d>& velocities,
                     vector<Vec2d>& accelerations) = 0;
}; 

class ParticleSystem {
public:
  ParticleSystem(const vector<Vec2d>& initialPositions, const vector<Vec2d>& initialVelocities,
                 vector<Force*> forces, vector<Damping*> damping,
                 const double particleMass, const double timeStep);
  const vector<Vec2d>& positions() const;
  double time() const;
  void integrate(double time);
  
private:
  void eulerStep(const double dt);
  void updateAccelerations();

  void step(const double dt);
  vector<Vec2d> forceAccelerations() const;
  vector<Vec2d> dampingAccelerations(const vector<Vec2d>& velocities) const;
  
  const size_t numberOfParticles_;
  vector<Vec2d> positions_, velocities_, accelerations_;
  vector<Force*> forces_; 
  vector<Damping*> dampings_; 
  const double particleMass_;
  double time_;

  // Used for time stepping
  const double timeStep_;
  vector<Vec2d> approxVelocities_, forceAccelerations_, dampingAccelerations_;
};

class PointGravity : public Force {
public:
  PointGravity(const Vec2d center, const double intensity);
  void apply(const double time, const double particleMass, const vector<Vec2d>& positions,
             vector<Vec2d>& accelerations);
private:
  Vec2d center_;
  double intensity_;
};

class LinearDamping : public Damping {
public:
  LinearDamping(const double intensity);
  void apply(const double time, const double particleMass, const vector<Vec2d>& velocities,
             vector<Vec2d>& accelerations);
private:
  double intensity_;
};

#endif