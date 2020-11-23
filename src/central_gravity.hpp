#ifndef CENTRAL_GRAVITY_HPP
#define CENTRAL_GRAVITY_HPP

#include "simulation.hpp"
#include "forces.hpp"

class CentralGravity : public Simulation {
public:
  CentralGravity(
      const size_t numberOfParticles = 1000,
      const double totalMass = 1.0,
      const Rectangle region = Rectangle{-1.0, -1.0, 1.0, 1.0},
      const double gravityConstant = 1.0,
      const double dampingConstant = 0.01,
      const double timeStep = 0.01);

  void increaseGravity();
  void decreaseGravity();
  void increaseDamping();
  void decreaseDamping();

private:
  PointGravity gravity_;
  LinearDamping damping_;
};


#endif