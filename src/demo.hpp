#ifndef DEMO_HPP
#define DEMO_HPP

#include "simulation.hpp"
#include "forces.hpp"
#include "pressure_force.hpp"

class CentralPotential : public Simulation {
public:
  CentralPotential(size_t numberOfParticles = 1000,
                   double totalMass = 1.0,
                   Rectangle region = {-1.0, -1.0, 1.0, 1.0},
                   double gravityConstant = 1.0,
                   double dampingConstant = 0.01);

  SimulationRunner& runner() override;
  const SimulationRunner& runner() const override;
  const ParticleSystem& state() const override;
  LinearDamping& damping();
  PointGravity& gravity();

private:
  PointGravity gravity_;
  LinearDamping damping_;
  ParticleSystem ps_;
  SimulationRunner runner_;
};

class ToyStar : public Simulation {
public:
  ToyStar(size_t numberOfParticles = 250,
          double totalMass = 1.0,
          Rectangle region = {-1.0, -1.0, 1.0, 1.0},
          double dampingConstant = 1.0,
          double smoothingLength = 0.1,
          double pressureConstant = 0.1);

  SimulationRunner& runner() override;
  const SimulationRunner& runner() const override;
  const ParticleSystem& state() const override;
  LinearDamping& damping();
  PointGravity& gravity();

private:
  PointGravity gravity_;
  GasPressure pressureFunction_;
  PressureForce pressureForce_;
  LinearDamping damping_;
  ParticleSystem ps_;
  SimulationRunner runner_;
};

#endif