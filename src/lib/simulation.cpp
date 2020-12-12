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

Simulation::Simulation(const ParticleSystem& ps, unique_ptr<TimeIntegrator> integrator,
                       double playbackSpeed, int fps) :
  ps_(ps),
  integrator_(std::move(integrator)),
  synchronizer_(),
  simulationSpeed_(playbackSpeed),
  fps_(fps),
  paused_(true)
{}

Simulation::Simulation(size_t numberOfParticles, double totalMass, Rectangle region) :
  Simulation(ParticleSystem(numberOfParticles, totalMass, region))
{}

const vector<Vec2d>& Simulation::positions() const
{
  return ps_.positions;
}

double Simulation::time() const
{
  return ps_.time;
}

void Simulation::computeNextFrame()
{
  if (!paused()) {
    integrator_->integrate(ps_, simulationSpeed_ / fps_);
  }
}

void Simulation::waitForNextFrame()
{
  if (!paused()) {
    synchronizer_.waitUntil(ps_.time, simulationSpeed_);
  }
}

double Simulation::targetSpeed() const
{
  return simulationSpeed_;
}

void Simulation::setTargetSpeed(double newSpeed)
{
  synchronize();
  simulationSpeed_ = newSpeed;
}

bool Simulation::paused() const
{
  return paused_;
}

void Simulation::togglePause()
{
  if (paused_) {
    synchronize();
  }
  paused_ = !paused_;
}

void Simulation::addForce(Force& force)
{
  ps_.addForce(force);
}

void Simulation::addDamping(Damping& damping)
{
  ps_.addDamping(damping);
}

void Simulation::synchronize()
{
  synchronizer_.synchronize(ps_.time);
}