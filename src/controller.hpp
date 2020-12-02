#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "demo.hpp"

class Controller {
public:
  Controller(Simulation& simulation, const ParameterAdjuster& speed,
             const ParameterAdjuster& damping, const ParameterAdjuster& gravity);
  Controller(CentralPotential& simulation);
  void handleKeyboardInput(char c) const;

private:
  Simulation& simulation_;
  const ParameterAdjuster& speed_, damping_, gravity_;
};


#endif