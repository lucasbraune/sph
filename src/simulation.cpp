#include "simulation.hpp"

Simulation::Simulation(const ParticleSystem& ps, const TimeController& tc) :
  ps_(ps), tc_(tc)
{}

void Simulation::computeNextFrame()
{
  ps_.integrate(tc_.timeUntilNextFrame());
}


void Simulation::waitForNextFrame() const
{
  tc_.waitUntil(ps_.time());
}

const ParticleSystem& Simulation::particleSystem() const
{
  return ps_;
}