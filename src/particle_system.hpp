#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include <memory>
#include <cmath>
#include <functional>
#include "util.hpp"

using std::vector;
using std::reference_wrapper;

class Force {
public:
  virtual ~Force() {}
  virtual void apply(const double time, const double particleMass, const vector<Vec2d>& positions,
                     vector<Vec2d>& accelerations) const = 0;
};

class PointGravity : public Force {
public:
  PointGravity(const double gravityConstant, const Vec2d center = ZERO_VECTOR);
  void apply(const double time, const double particleMass, const vector<Vec2d>& positions,
             vector<Vec2d>& accelerations) const;
  double constant() const;
  void setConstant(double intensity);
  
private:
  Vec2d center_;
  double intensity_;
};

class Damping {
public:
  virtual ~Damping() {}
  virtual Vec2d acceleration(const double time, const double mass, const Vec2d velocity) const = 0;
};

class LinearDamping : public Damping {
public:
  LinearDamping(const double dampingConstant);
  Vec2d acceleration(const double time, const double mass, const Vec2d velocity) const;
  double constant() const;
  void setConstant(double newValue);

private:
  double intensity_;
};

struct ParticleSystem {
  ParticleSystem(const vector<Vec2d>& initialPositions, const vector<Vec2d>& initialVelocities,
                 const vector<reference_wrapper<const Force>>& forces, const Damping& damping,
                 double particleMass);
  ParticleSystem(size_t numberOfParticles, double totalMass, Rectangle region,
                 const vector<reference_wrapper<const Force>>& forces, const Damping& damping);
  const size_t numberOfParticles;
  const double particleMass;
  const vector<reference_wrapper<const Force>> forces;
  const Damping& damping; 
  vector<Vec2d> positions, velocities, accelerations;
  double time;
};

class TimeIntegrator {
public:
  virtual ~TimeIntegrator() {};
  virtual void step(ParticleSystem& ps) = 0;
  void integrate(ParticleSystem& ps, double duration);
};

class EulerIntegrator : public TimeIntegrator {
public:
  EulerIntegrator(double timeStep);
  void step(ParticleSystem& ps) override;

private:
  const double timeStep_;
};

class VerletIntegrator : public TimeIntegrator {
public:
  VerletIntegrator(double timeStep);
  void step(ParticleSystem& ps) override;

private:
  static inline Vec2d nextVelocity(Vec2d currVel, Vec2d currAcc, Vec2d nextForceAcc,
                                   const Damping& damping, double mass, double time,
                                   double timeStep);
  const double timeStep_;
  vector<Vec2d> nextForceAcc_;
};

#endif