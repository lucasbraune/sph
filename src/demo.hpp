#ifndef DEMO_HPP
#define DEMO_HPP

#include "simulation.hpp"
#include "pressure_force.hpp"

class ParameterAdjuster {
public:
  ParameterAdjuster(const function<double(void)>& getter, const function<void(double)>& setter);
  ParameterAdjuster();
  void increase() const;
  void decrease() const;

private:
  function<double(void)> get_;
  function<void(double)> set_;
};

namespace AdjusterFactory {
  ParameterAdjuster speed(Simulation& runner);
  ParameterAdjuster damping(LinearDamping& damping);
  ParameterAdjuster gravity(PointGravity& gravity);
};

class CentralPotential : public Simulation {
public:
  CentralPotential(size_t numberOfParticles = 1000,
                   double totalMass = 1.0,
                   Rectangle region = {-1.0, -1.0, 1.0, 1.0},
                   double gravityConstant = 1.0,
                   double dampingConstant = 0.01);

  const ParameterAdjuster& speedAdjuster();
  const ParameterAdjuster& dampingAdjuster();
  const ParameterAdjuster& gravityAdjuster();

private:
  PointGravity gravity_;
  LinearDamping damping_;
  ParameterAdjuster speedAdjuster_, dampingAdjuster_, gravityAdjuster_;
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