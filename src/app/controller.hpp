#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "simulation.hpp"
#include "sample_simulations.hpp"

void handleKeyboardInput(sph::SimulationInterface& sim, char c);
void handleKeyboardInput(sph::BreakingDamSimulation& sim, char c);

#endif