/*
 * File: physics_elements.hpp
 * 
 * A "physics element" is an implementation of Force, Damping or Collidable, three interfaces 
 * defined in particle_system.hpp. This header and its implementation file define basic physics 
 * elements, which are used as building blocks for simulations in the files
 * sample_simulations.hpp/cpp.
 */

#ifndef PS_ELEMENTS_HPP
#define PS_ELEMENTS_HPP

#include "particle_system.hpp"

namespace sph {

class PointGravity : public Force {
public:
  PointGravity(double gravityConstant, const Vec2d& center = ZERO_VECTOR);
  void apply(double time, double particleMass, const std::vector<Vec2d>& positions,
             std::vector<Vec2d>& accelerations) const override;
  double constant() const;
  void setConstant(double intensity);
  
private:
  Vec2d center_;
  double intensity_;
};

class SurfaceGravity : public Force {
public:
  SurfaceGravity(double magnitude) : acceleration_{0.0, -magnitude} {}
  void apply(double time, double particleMass, const std::vector<Vec2d>& positions,
             std::vector<Vec2d>& accelerations) const override;
  double magnitude() const { return -acceleration_[1]; }
  void setMagnitude(double newValue) { acceleration_[1] = -newValue; }
  
private:
  Vec2d acceleration_;
};

class LinearDamping : public Damping {
public:
  LinearDamping(double dampingConstant);
  Vec2d acceleration(double time, double mass, const Vec2d& velocity) const override;
  double constant() const;
  void setConstant(double newValue);

private:
  double intensity_;
};

class Wall : public Collidable {
public:
  Wall(const Vec2d& normal, // must be nonzero
       double distanceFromTheOrigin);
  void resolveCollisions(std::vector<Vec2d>& positions, std::vector<Vec2d>& velocities,
                         double time) const override;

private:
  void resolveCollision(Vec2d& pos, Vec2d& vel) const;
  Vec2d unitNormal_; // points out of the wall
  Vec2d ptOnWall_; 
};

} // end namespace sph

#endif