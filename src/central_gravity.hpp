#ifndef CENTRAL_GRAVITY_HPP
#define CENTRAL_GRAVITY_HPP

#include "simulation.hpp"
#include "forces.hpp"

class CentralGravity {
public:
  CentralGravity(
      const size_t numberOfParticles = 1000,
      const double totalMass = 1.0,
      const Rectangle region = {-1.0, -1.0, 1.0, 1.0},
      const double gravityConstant = 1.0,
      const double dampingConstant = 0.01);

  Simulation& simulation();
  LinearDamping& damping();
  PointGravity& gravity();

private:
  PointGravity gravity_;
  LinearDamping damping_;
  ParticleSystem ps_;
  Simulation sim_;
};

#endif