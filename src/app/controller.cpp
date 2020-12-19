#include "controller.hpp"

using namespace sph;

static void helper(SimulationInterface& simulation, char c)
{
  switch (c) {
  case 'p':
  case 'P':
    simulation.togglePause();
    break;
  case 's':
    simulation.decreaseTargetSpeed();
    break;
  case 'S':
    simulation.increaseTargetSpeed();
    break;
  }
}

void handleKeyboardInput(SimulationInterface& simulation, char c)
{
  return helper(simulation, c);
}

void handleKeyboardInput(BreakingDamSimulation& simulation, char c)
{
  switch (c) {
  case 'b':
  case 'B':
    simulation.breakDam();
    break;
  case 'd':
    simulation.decreaseDamping();
    break;
  case 'D':
    simulation.increaseDamping();
    break;
  default:
    helper(simulation, c);
  }
}