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
  virtual void apply(const vector<Vec2d>& positions, const vector<Vec2d>& velocities,  
      vector<Vec2d>& accelerations, const double t, const double particleMass) = 0;
};

class PointGravity : public Force {
public:
  PointGravity(const Vec2d center, const double nu);
  void apply(const vector<Vec2d>& positions, const vector<Vec2d>& velocities,  
      vector<Vec2d>& accelerations, const double t, const double particleMass);

private:
  Vec2d center_;
  double nu_;
};

class ParticleSystem {
public:
  ParticleSystem(const vector<Vec2d>& x, const vector<Vec2d>& v, vector<Force*> forces,
                 const double m, const double dt);
  const vector<Vec2d>& positions() const;
  double time() const;
  void integrate(double time);
  
private:
  void eulerStep(const double dt);
  void updateAccelerations();
  
  const size_t N_;
  vector<Vec2d> positions_, velocities_, accelerations_;
  vector<Force*> forces_; 
  const double particleMass_;
  double t_;
  const double dt_;
};

#endif