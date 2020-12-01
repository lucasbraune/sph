#include "simulation.hpp"
#include <thread>

using std::chrono::milliseconds;

Synchronizer::Synchronizer() :
  fixedRealTime_(high_resolution_clock::now()),
  fixedSimTime_(0.0)
{}

milliseconds toMilliseconds(double seconds)
{
  return milliseconds((long) (seconds * 1e3));
}

void Synchronizer::synchronize(const double simulationTime)
{
  fixedRealTime_ = high_resolution_clock::now();
  fixedSimTime_ = simulationTime;
}

void Synchronizer::waitUntil(const double simulationTime, const double playbackSpeed) const 
{
  double difference = simulationTime - fixedSimTime_;
  auto realTime = fixedRealTime_ + toMilliseconds(difference / playbackSpeed);
  std::this_thread::sleep_until(realTime);
}

SimulationRunner::SimulationRunner(ParticleSystem& ps, unique_ptr<TimeIntegrator> integrator, double playbackSpeed, int fps) :
  ps_(ps),
  integrator_(std::move(integrator)),
  synchronizer_(),
  simulationSpeed_(playbackSpeed),
  fps_(fps),
  paused_(true)
{}

void SimulationRunner::computeNextFrame()
{
  if (!paused()) {
    integrator_->integrate(ps_, simulationSpeed_ / fps_);
  }
}

void SimulationRunner::waitForNextFrame() const
{
  if (!paused()) {
    synchronizer_.waitUntil(ps_.time, simulationSpeed_);
  }
}

double SimulationRunner::targetSpeed() const
{
  return simulationSpeed_;
}

void SimulationRunner::speedUp()
{
  synchronize();
  simulationSpeed_ *= 2.0;
}

void SimulationRunner::speedDown()
{
  synchronize();
  simulationSpeed_ *= 0.5;
}

bool SimulationRunner::paused() const
{
  return paused_;
}

void SimulationRunner::pauseOrUnpause()
{
  if (paused_) {
    synchronize();
  }
  paused_ = !paused_;
}

void SimulationRunner::synchronize()
{
  synchronizer_.synchronize(ps_.time);
}