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
  ps_(ps), timeUtil_(), playbackSpeed_(playbackSpeed), fps_(fps)
{}

void Simulation::synchronize()
{
  timeUtil_.synchronize(ps_.time());
}

void Simulation::computeNextFrame()
{
  ps_.integrate(playbackSpeed_ / fps_);
}

void Simulation::waitForNextFrame() const
{
  timeUtil_.waitUntil(ps_.time(), playbackSpeed_);
}

double Simulation::playbackSpeed() const
{
  return playbackSpeed_;
}

void Simulation::setPlaybackSpeed(const double playbackSpeed)
{
  playbackSpeed_ = playbackSpeed;
}

const vector<Vec2d>& Simulation::positions() const
{
  return ps_.positions();
}

double Simulation::time() const
{
  return ps_.time();
}