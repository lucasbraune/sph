#ifndef EXAMPLES_HPP
#define EXAMPLES_HPP

#include "simulation.hpp"
#include "pre_physics.hpp"
#include "pressure_force.hpp"

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

struct CentralGravity : public PrePhysics {
  CentralGravity(double gravityConstant, double dampingConstant);

  const vector<const Force*> createForceVector() const {return {&gravity_}; }
  const vector<const Damping*> createDampingVector() const {return {&damping_}; }

  PointGravity gravity_;
  LinearDamping damping_;
};

struct ToyStar : public PrePhysics {
  ToyStar(double gravityConstant,
          double dampingConstant,
          double pressureConstant,
          double interactionRadius);

  const vector<const Force*> createForceVector() const {return {&gravity_, &pressure_}; }
  const vector<const Damping*> createDampingVector() const {return {&damping_}; }

  PointGravity gravity_;
  LinearDamping damping_;
  PressureForce pressure_;
};

Simulation<PhysicsAdapter<CentralGravity>> createCentralGravitySimulation(
    size_t numberOfParticles = 1000,
    double totalMass = 1.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityConstant = 1.0,
    double dampingConstant = 0.01,
    double timeStep = 0.01);

Simulation<PhysicsAdapter<ToyStar>> createToyStarSimulation(
    size_t numberOfParticles = 250,
    double starMass = 2.0,
    double starRadius = 0.75,
    Rectangle initialRegion = {-1.0, -1.0, 1.0, 1.0},
    double dampingConstant = 1.0,
    double pressureConstant = 1.0,
    double timeStep = 0.01);

#endif