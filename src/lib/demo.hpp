#ifndef DEMO_HPP
#define DEMO_HPP

#include "simulation.hpp"
#include "pre_physics.hpp"
#include "pressure_force.hpp"

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
    double dampingConstant = 0.01);

Simulation<PhysicsAdapter<ToyStar>> createToyStarSimulation(
    size_t numberOfParticles = 250,
    double starMass = 2.0,
    double starRadius = 0.75,
    Rectangle initialRegion = {-1.0, -1.0, 1.0, 1.0},
    double dampingConstant = 1.0,
    double pressureConstant = 1.0);

#endif