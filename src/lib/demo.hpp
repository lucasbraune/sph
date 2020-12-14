#ifndef DEMO_HPP
#define DEMO_HPP

#include "simulation.hpp"
#include "pressure_force.hpp"

class Physics {
public:
  ~Physics() {}
  const vector<const Force*>& forces() const { return forces_; }
  const vector<const Damping*>& dampings() const { return dampings_; }

protected:
  Physics() {}
  vector<const Force*> forces_;
  vector<const Damping*> dampings_;
};

class NoPhysics : public Physics {
  NoPhysics() {}
};

class CentralGravityPhysics : public Physics {
public:
  CentralGravityPhysics(double gravityConstant,
                        double dampingConstant);
private:
  PointGravity gravity_;
  LinearDamping damping_;
};

class ToyStarPhysics : public Physics {
public:
  ToyStarPhysics(double gravityConstant,
                 double dampingConstant,
                 double pressureConstant,
                 double interactionRadius);
private:
  PointGravity gravity_;
  LinearDamping damping_;
  PressureForce pressure_;
};

class CentralPotential : public Simulation {
public:
  CentralPotential(size_t numberOfParticles = 1000,
                   double totalMass = 1.0,
                   Rectangle region = {-1.0, -1.0, 1.0, 1.0},
                   double gravityConstant = 1.0,
                   double dampingConstant = 0.01);
private:
  PointGravity gravity_;
  LinearDamping damping_;
};

class ToyStar : public CentralPotential {
public:
  ToyStar(size_t numberOfParticles = 250,
          double starMass = 2.0,
          double starRadius = 0.75,
          Rectangle initialRegion = {-1.0, -1.0, 1.0, 1.0},
          double dampingConstant = 1.0,
          double pressureConstant = 1.0);

private:
  PressureForce pressureForce_;
};

#endif