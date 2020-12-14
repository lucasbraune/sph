#ifndef DEMO_HPP
#define DEMO_HPP

#include "simulation.hpp"
#include "pressure_force.hpp"

class Physics {
public:
  virtual ~Physics() {}
  const vector<const Force*>& forces() const { return forces_; }
  const vector<const Damping*>& dampings() const { return dampings_; }

protected:
  Physics() {}
  vector<const Force*> forces_;
  vector<const Damping*> dampings_;
};

class CentralGravityPhysics : public Physics {
public:
  CentralGravityPhysics(double gravityConstant,
                        double dampingConstant);
  CentralGravityPhysics(const CentralGravityPhysics& other);
  CentralGravityPhysics(CentralGravityPhysics&& other);
  CentralGravityPhysics& operator=(const CentralGravityPhysics& other);
  CentralGravityPhysics& operator=(CentralGravityPhysics&& other);
  ~CentralGravityPhysics() = default;

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
  ToyStarPhysics(const ToyStarPhysics& other);
  ToyStarPhysics(ToyStarPhysics&& other);
  ToyStarPhysics& operator=(const ToyStarPhysics& other);
  ToyStarPhysics& operator=(ToyStarPhysics&& other);
  ~ToyStarPhysics() = default;

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