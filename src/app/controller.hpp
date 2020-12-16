#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "examples.hpp"

class Controller {
public:
  Controller(sph::SimulationInterface& simulation);
  void handleKeyboardInput(char c) const;

private:
  sph::SimulationInterface& simulation_;
};


#endif