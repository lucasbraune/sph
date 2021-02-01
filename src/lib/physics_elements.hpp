/*
 * File: physics_elements.hpp
 * 
 * This file declares the Force, Damping and Collidable interfaces, each of which
 * isolates a functionality from the complicated Physics interface defined in the file
 * particle_system.hpp.
 * 
 * Sample implementataions of Force, Damping and Collidable are provided. Such
 * implementations can be used as bulding blocks for implementations of Physics interface;
 * see sample_simulations.hpp/cpp for examples.
 */

#ifndef PS_ELEMENTS_HPP
#define PS_ELEMENTS_HPP

#include "particle_system.hpp"

namespace sph {

struct Force {
  virtual ~Force() {}
  virtual void apply(ParticleSystem& ps) const = 0;
};

struct Damping {
  virtual ~Damping() {}
  virtual void apply(ParticleSystem& ps) const = 0;
};

struct Collidable {
  virtual ~Collidable() {}
  virtual void resolveCollisions(ParticleSystem& ps) const = 0;
};

class PointGravity : public Force {
public:
  PointGravity(double gravityConstant, const Vec2d& center = {});
  void apply(ParticleSystem& ps) const final;
  double constant() const { return intensity_; }
  void setConstant(double intensity) { intensity_ = intensity; }

private:
  Vec2d center_;
  double intensity_;
};

class SurfaceGravity : public Force {
public:
  SurfaceGravity(double magnitude);
  void apply(ParticleSystem& ps) const final;
  double magnitude() const { return -acceleration_[1]; }
  void setMagnitude(double newValue) { acceleration_[1] = -newValue; }
  
private:
  Vec2d acceleration_;
};

class LinearDamping : public Damping {
public:
  LinearDamping(double dampingConstant);
  void apply(ParticleSystem& ps) const final;
  double constant() const { return intensity_; }
  void setConstant(double newValue) { intensity_ = newValue; }

private:
  double intensity_; // force per unit velocity
};

class Wall : public Collidable {
public:
  Wall(const Vec2d& normal, // must be nonzero
       double distanceFromTheOrigin);
  void resolveCollisions(ParticleSystem& ps) const final;
  void move(const Vec2d& displacement) { ptOnWall_ += displacement; }

private:
  void resolveCollision(Vec2d& pos, Vec2d& vel) const;
  Vec2d unitNormal_; // points out of the wall
  Vec2d ptOnWall_; 
};

} // end namespace sph

#endif