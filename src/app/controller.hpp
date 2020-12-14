#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "demo.hpp"

class Controller {
public:
  Controller(Simulation& simulation);
  void handleKeyboardInput(char c) const;

private:
  Simulation& simulation_;
};


#endif