#include "controller.hpp"

using namespace sph;

Controller::Controller(SimulationInterface& simulation) :
  simulation_(simulation)
{}

void Controller::handleKeyboardInput(char c) const
{
  switch (c) {
  case 'p':
    // Fall through
  case 'P':
    simulation_.togglePause();
    break;
  case 's':
    simulation_.decreaseTargetSpeed();
    break;
  case 'S':
    simulation_.increaseTargetSpeed();
    break;
  }
}