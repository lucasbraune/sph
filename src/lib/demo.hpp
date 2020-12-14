#ifndef DEMO_HPP
#define DEMO_HPP

#include "simulation.hpp"
#include "pressure_force.hpp"

class PhysicsInterface {
public:
  virtual ~PhysicsInterface() {}
  // TODO: Make the following two methods return references.
  virtual const vector<const Force*> forces() const = 0;
  virtual const vector<const Damping*> dampings() const = 0;
};

class NoPhysics : public PhysicsInterface {
  NoPhysics() {}
  const vector<const Force*> forces() const { return {}; }
  const vector<const Damping*> dampings() const { return {}; }
private:
  vector<const Force*> forces_;
  vector<const Damping*> dampings_;
};

class CentralGravityPhysics : public PhysicsInterface {
public:
  CentralGravityPhysics(double gravityConstant,
                        double dampingConstant);

  const vector<const Force*> forces() const { return {&gravity_}; }
  const vector<const Damping*> dampings() const { return {&damping_}; }

private:
  PointGravity gravity_;
  LinearDamping damping_;
};

class ToyStarPhysics : public PhysicsInterface {
public:
  ToyStarPhysics(double gravityConstant,
                 double dampingConstant,
                 double pressureConstant,
                 double interactionRadius);
  const vector<const Force*> forces() const { return {&gravity_, &pressure_}; }
  const vector<const Damping*> dampings() const { return {&damping_}; }

private:
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