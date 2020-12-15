#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include <memory>
#include <cmath>
#include "util.hpp"

using std::vector;

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
  virtual void resolveCollision(Vec2d& pos, Vec2d& vel, double time) const = 0;
};

struct Physics {
  virtual ~Physics() {}
  virtual const vector<const Force*>& forcePtrs() const = 0;
  virtual const vector<const Damping*>& dampingPtrs() const = 0;
};

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

struct TimeIntegrator {
  virtual ~TimeIntegrator() {};
  virtual void step(ParticleSystem& ps,
                    const vector<const Force*>& forces,
                    const vector<const Damping*>& dampings) = 0;
  void integrate(ParticleSystem& ps,
                 const vector<const Force*>& forces,
                 const vector<const Damping*>& dampings,
                 double duration);
};

class PointGravity : public Force {
public:
  PointGravity(double gravityConstant, const Vec2d& center = ZERO_VECTOR);
  void apply(double time, double particleMass, const vector<Vec2d>& positions,
             vector<Vec2d>& accelerations) const;
  double constant() const;
  void setConstant(double intensity);
  
private:
  Vec2d center_;
  double intensity_;
};

class LinearDamping : public Damping {
public:
  LinearDamping(double dampingConstant);
  Vec2d acceleration(double time, double mass, const Vec2d& velocity) const;
  double constant() const;
  void setConstant(double newValue);

private:
  double intensity_;
};

class Wall : public Collidable {
public:
  Wall(const Vec2d& unitNormal, const Vec2d& ptOnWall);
  void resolveCollision(Vec2d& pos, Vec2d& vel, double) const;

private:
  Vec2d unitNormal_, ptOnWall_; 
};

class EulerIntegrator : public TimeIntegrator {
public:
  EulerIntegrator(double timeStep);
  void step(ParticleSystem& ps,
            const vector<const Force*>& forces,
            const vector<const Damping*>& dampings) override;

private:
  const double timeStep_;
};

class VerletIntegrator : public TimeIntegrator {
public:
  VerletIntegrator(double timeStep);
  void step(ParticleSystem& ps,
            const vector<const Force*>& forces,
            const vector<const Damping*>& dampings) override;

private:
  static inline Vec2d nextVelocity(Vec2d currVel, Vec2d currAcc, Vec2d nextForceAcc,
                                   const vector<const Damping*>& dampings, double mass, double time,
                                   double timeStep);
  const double timeStep_;
  vector<Vec2d> nextForceAcc_;
};

#endif