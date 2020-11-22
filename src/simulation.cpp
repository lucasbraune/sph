#include "simulation.hpp"
#include <thread>

using std::chrono::milliseconds;

TimeController::TimeController(int targetFps, double playbackSpeed) :
  sim_start_(high_resolution_clock::now()),
  fps_(targetFps),
  playbackSpeed_(playbackSpeed)
{}

TimeController::TimeController(int targetFps) :
  TimeController(targetFps, 1.0)
{}

void TimeController::setStartOfSimulation()
{
  sim_start_ = high_resolution_clock::now();
}

milliseconds toMilliseconds(double seconds)
{
  return milliseconds((long) (seconds * 1e3));
}

void TimeController::waitUntil(double simulationTime) const 
{
  auto realTime = sim_start_ + toMilliseconds(simulationTime / playbackSpeed_);
  std::this_thread::sleep_until(realTime);
}

double TimeController::timeUntilNextFrame() const 
{
  return playbackSpeed_ / fps_;
}


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