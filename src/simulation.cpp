#include "simulation.hpp"
#include <thread>

using std::chrono::milliseconds;

TimeUtil::TimeUtil() :
  fixedRealTime_(high_resolution_clock::now()),
  fixedSimTime_(0.0)
{}

milliseconds toMilliseconds(double seconds)
{
  return milliseconds((long) (seconds * 1e3));
}

void TimeUtil::synchronize(const double simulationTime)
{
  fixedRealTime_ = high_resolution_clock::now();
  fixedSimTime_ = simulationTime;
}

void TimeUtil::waitUntil(const double simulationTime, const double playbackSpeed) const 
{
  double difference = simulationTime - fixedSimTime_;
  auto realTime = fixedRealTime_ + toMilliseconds(difference / playbackSpeed);
  std::this_thread::sleep_until(realTime);
}


Simulation::Simulation(const ParticleSystem& ps, const double playbackSpeed, const int fps) :
  ps_(ps), timeUtil_(), simulationSpeed_(playbackSpeed), fps_(fps), paused_(true)
{}

void Simulation::computeNextFrame()
{
  if (!paused()) {
    ps_.integrate(simulationSpeed_ / fps_);
  }
}

void Simulation::waitForNextFrame() const
{
  if (!paused()) {
    timeUtil_.waitUntil(ps_.time(), simulationSpeed_);
  }
}

double Simulation::speed() const
{
  return simulationSpeed_;
}

void Simulation::speedUp()
{
  synchronize();
  simulationSpeed_ *= 2.0;
}

void Simulation::speedDown()
{
  synchronize();
  simulationSpeed_ *= 0.5;
}

bool Simulation::paused() const
{
  return paused_;
}

void Simulation::switchPauseState()
{
  if (paused_) {
    synchronize();
  }
  paused_ = !paused_;
}

const vector<Vec2d>& Simulation::positions() const
{
  return ps_.positions();
}

double Simulation::time() const
{
  return ps_.time();
}

void Simulation::synchronize()
{
  timeUtil_.synchronize(ps_.time());
}