#include "simulation.hpp"
#include <thread>

using std::chrono::milliseconds;

Synchronizer::Synchronizer() :
  fixedRealTime_(high_resolution_clock::now()),
  fixedSimTime_(0.0)
{}

static milliseconds toMilliseconds(double seconds)
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

SimulationRunner::SimulationRunner(const ParticleSystem& ps, unique_ptr<TimeIntegrator> integrator, double playbackSpeed, int fps) :
  ps_(ps),
  integrator_(std::move(integrator)),
  synchronizer_(),
  simulationSpeed_(playbackSpeed),
  fps_(fps),
  paused_(true)
{}

SimulationRunner::SimulationRunner(size_t numberOfParticles, double totalMass, Rectangle region) :
  SimulationRunner(ParticleSystem(numberOfParticles, totalMass, region))
{}

const vector<Vec2d>& SimulationRunner::positions() const
{
  return ps_.positions;
}

double SimulationRunner::time() const
{
  return ps_.time;
}

void SimulationRunner::computeNextFrame()
{
  if (!paused()) {
    integrator_->integrate(ps_, simulationSpeed_ / fps_);
  }
}

void SimulationRunner::waitForNextFrame()
{
  if (!paused()) {
    synchronizer_.waitUntil(ps_.time, simulationSpeed_);
  }
}

double SimulationRunner::targetSpeed() const
{
  return simulationSpeed_;
}

void SimulationRunner::setTargetSpeed(double newSpeed)
{
  synchronize();
  simulationSpeed_ = newSpeed;
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

void SimulationRunner::addForce(Force& force)
{
  ps_.addForce(force);
}

void SimulationRunner::addDamping(Damping& damping)
{
  ps_.addDamping(damping);
}

void SimulationRunner::synchronize()
{
  synchronizer_.synchronize(ps_.time);
}