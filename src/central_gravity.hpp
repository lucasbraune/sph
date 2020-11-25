#ifndef CENTRAL_GRAVITY_HPP
#define CENTRAL_GRAVITY_HPP

#include "simulation.hpp"
#include "forces.hpp"

class CentralGravitySimulation {
public:
  CentralGravitySimulation(
      const size_t numberOfParticles = 1000,
      const double totalMass = 1.0,
      const Rectangle region = {-1.0, -1.0, 1.0, 1.0},
      const double gravityConstant = 1.0,
      const double dampingConstant = 0.01);

  SimulationRunner& runner();
  const ParticleSystem& state() const;
  LinearDamping& damping();
  PointGravity& gravity();

private:
  PointGravity gravity_;
  LinearDamping damping_;
  ParticleSystem ps_;
  SimulationRunner runner_;
};

#endif