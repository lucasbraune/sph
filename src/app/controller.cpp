#include "controller.hpp"

Controller::Controller(Simulation& simulation) :
  simulation_(simulation)
{}

void Controller::handleKeyboardInput(char c) const
{
  switch (c) {
  case 'p':
  case 'P':
    simulation_.togglePause();
    break;
  case 's':
    simulation_.setTargetSpeed(0.5 * simulation_.targetSpeed());
    break;
  case 'S':
    simulation_.setTargetSpeed(2.0 * simulation_.targetSpeed());
    break;
  }
}