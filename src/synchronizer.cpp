#include "synchronizer.hpp"
#include <thread>

using std::chrono::milliseconds;

Synchronizer::Synchronizer(int targetFps, double playbackSpeed) :
  sim_start_(high_resolution_clock::now()),
  fps_(targetFps),
  playbackSpeed_(playbackSpeed)
{}

Synchronizer::Synchronizer(int targetFps) :
  Synchronizer(targetFps, 1.0)
{}

void Synchronizer::setStart()
{
  sim_start_ = high_resolution_clock::now();
}

milliseconds toMilliseconds(double seconds)
{
  return milliseconds((long) (seconds * 1e3));
}

void Synchronizer::waitUntil(double simulationTime) const 
{
  auto realTime = sim_start_ + toMilliseconds(simulationTime / playbackSpeed_);
  std::this_thread::sleep_until(realTime);
}

double Synchronizer::timeUntilNextFrame() const 
{
  return playbackSpeed_ / fps_;
}