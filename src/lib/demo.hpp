#ifndef DEMO_HPP
#define DEMO_HPP

#include "simulation.hpp"
#include "pressure_force.hpp"

struct PhysicsInterface {
  virtual ~PhysicsInterface() {}
  virtual const vector<const Force*> createForceVector() const = 0;
  virtual const vector<const Damping*> createDampingVector() const = 0;
};

struct CentralGravityPhysics : public PhysicsInterface {
  CentralGravityPhysics(double gravityConstant,
                 double dampingConstant);

  const vector<const Force*> createForceVector() const {return {&gravity_}; }
  const vector<const Damping*> createDampingVector() const {return {&damping_}; }

  PointGravity gravity_;
  LinearDamping damping_;
};

struct ToyStarPhysics : public PhysicsInterface {
  ToyStarPhysics(double gravityConstant,
                 double dampingConstant,
                 double pressureConstant,
                 double interactionRadius);

  const vector<const Force*> createForceVector() const {return {&gravity_, &pressure_}; }
  const vector<const Damping*> createDampingVector() const {return {&damping_}; }

  PointGravity gravity_;
  LinearDamping damping_;
  PressureForce pressure_;
};

Simulation<CentralGravityPhysics> createCentralGravitySimulation(
    size_t numberOfParticles = 1000,
    double totalMass = 1.0,
    Rectangle region = {-1.0, -1.0, 1.0, 1.0},
    double gravityConstant = 1.0,
    double dampingConstant = 0.01);

Simulation<ToyStarPhysics> createToyStarSimulation(
    size_t numberOfParticles = 250,
    double starMass = 2.0,
    double starRadius = 0.75,
    Rectangle initialRegion = {-1.0, -1.0, 1.0, 1.0},
    double dampingConstant = 1.0,
    double pressureConstant = 1.0);

#endif