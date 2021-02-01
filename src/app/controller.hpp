#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "simulation.hpp"
#include "sample_simulations.hpp"

namespace detail {

template<class Simulation>
void handleBasicInput(Simulation& simulation, char c)
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

} // namespace detail

template<class Simulation>
void handleKeyboardInput(Simulation& simulation, char c)
{
  detail::handleBasicInput(simulation, c);
}

template<>
void handleKeyboardInput(sph::BreakingDamSimulation& simulation, char c)
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
    detail::handleBasicInput(simulation, c);
  }
}

#endif