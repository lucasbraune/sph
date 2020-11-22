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

const vector<Vec2d>& Simulation::positions() const
{
  return ps_.positions();
}

double Simulation::time() const
{
  return ps_.time();
}