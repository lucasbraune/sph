#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "examples.hpp"

class Controller {
public:
  Controller(SimulationInterface& simulation);
  void handleKeyboardInput(char c) const;

private:
  SimulationInterface& simulation_;
};


#endif