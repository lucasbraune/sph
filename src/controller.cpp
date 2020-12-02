#include "controller.hpp"

Controller::Controller(Simulation& simulation, const ParameterAdjuster& speed,
                       const ParameterAdjuster& damping, const ParameterAdjuster& gravity) :
  simulation_(simulation),
  speed_(speed),
  damping_(damping),
  gravity_(gravity)
{}

Controller::Controller(CentralPotential& simulation) :
  Controller(simulation,
             simulation.speedAdjuster(),
             simulation.dampingAdjuster(),
             simulation.gravityAdjuster())
{}

Controller::Controller(ToyStar& simulation) :
  Controller(simulation,
             simulation.speedAdjuster(),
             simulation.dampingAdjuster(),
             simulation.gravityAdjuster())
{}

void Controller::handleKeyboardInput(char c) const
{
  switch (c) {
  case 'd':
    damping_.decrease();
    break;
  case 'D':
    damping_.increase();
    break;
  case 'g':
    gravity_.decrease();
    break;
  case 'G':
    gravity_.increase();
    break;
  case 'p':
  case 'P':
    simulation_.pauseOrUnpause();
    break;
  case 's':
    speed_.decrease();
    break;
  case 'S':
    speed_.increase();
    break;
  }
}