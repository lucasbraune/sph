#ifndef DEMO_HPP
#define DEMO_HPP

#include "simulation.hpp"
#include "pressure_force.hpp"

class ParameterAdjuster {
public:
  ParameterAdjuster(const function<double(void)>& getter, const function<void(double)>& setter);
  void increase() const;
  void decrease() const;

private:
  function<double(void)> get_;
  function<void(double)> set_;
};

namespace AdjusterFactory {
  ParameterAdjuster speed(SimulationRunner& runner);
  ParameterAdjuster gravity(PointGravity& gravity);
};

class CentralPotential {
public:
  CentralPotential(size_t numberOfParticles = 1000,
                   double totalMass = 1.0,
                   Rectangle region = {-1.0, -1.0, 1.0, 1.0},
                   double gravityConstant = 1.0,
                   double dampingConstant = 0.01);

  const SimulationRunner& runner() const;
  SimulationRunner& runner();
  LinearDamping& damping();
  const ParameterAdjuster& speedAdjuster();
  const ParameterAdjuster& gravityAdjuster();

private:
  PointGravity gravity_;
  LinearDamping damping_;
  ParticleSystem ps_;
  SimulationRunner runner_;
  ParameterAdjuster speedAdjuster_, gravityAdjuster_;
};

class ToyStar {
public:
  ToyStar(size_t numberOfParticles = 250,
          double starMass = 2.0,
          double starRadius = 0.75,
          Rectangle initialRegion = {-1.0, -1.0, 1.0, 1.0},
          double dampingConstant = 1.0,
          double pressureConstant = 1.0);

  SimulationRunner& runner();
  const SimulationRunner& runner() const;
  LinearDamping& damping();
  const ParameterAdjuster& speedAdjuster();
  const ParameterAdjuster& gravityAdjuster();

private:
  PointGravity gravity_;
  PressureForce pressureForce_;
  LinearDamping damping_;
  ParticleSystem ps_;
  SimulationRunner runner_;
  ParameterAdjuster speedAdjuster_, gravityAdjuster_;
};

#endif