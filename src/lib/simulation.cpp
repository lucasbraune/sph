#include "simulation.hpp"

#include <thread>

using std::chrono::milliseconds;

Synchronizer::Synchronizer()
    : fixedRealTime_(high_resolution_clock::now()), fixedSimTime_(0.0) {}

static milliseconds toMilliseconds(double seconds) {
  return milliseconds((long)(seconds * 1e3));
}

void Synchronizer::synchronize(const double simulationTime) {
  fixedRealTime_ = high_resolution_clock::now();
  fixedSimTime_ = simulationTime;
}

void Synchronizer::waitUntil(const double simulationTime,
                             const double playbackSpeed) const {
  double difference = simulationTime - fixedSimTime_;
  auto realTime = fixedRealTime_ + toMilliseconds(difference / playbackSpeed);
  std::this_thread::sleep_until(realTime);
}

Simulation::Simulation(const ParticleSystem& ps,
                       const vector<const Force*>& forces,
                       const vector<const Damping*>& dampings,
                       unique_ptr<TimeIntegrator> integrator,
                       double playbackSpeed, int fps)
    : ps_(ps),
      forces_(forces),
      dampings_(dampings),
      integrator_(std::move(integrator)),
      synchronizer_(),
      simulationSpeed_(playbackSpeed),
      fps_(fps),
      paused_(true) {}

Simulation::Simulation(size_t numberOfParticles, double totalMass,
                       Rectangle region)
    : Simulation(ParticleSystem(numberOfParticles, totalMass, region), {}, {}) {}

const vector<Vec2d>& Simulation::positions() const { return ps_.positions; }

double Simulation::time() const { return ps_.time; }

void Simulation::computeNextFrame() {
  if (!paused()) {
    integrator_->integrate(ps_, forces_, dampings_, simulationSpeed_ / fps_);
  }
}

void Simulation::waitForNextFrame() {
  if (!paused()) {
    synchronizer_.waitUntil(ps_.time, simulationSpeed_);
  }
}

double Simulation::targetSpeed() const { return simulationSpeed_; }

void Simulation::setTargetSpeed(double newSpeed) {
  synchronize();
  simulationSpeed_ = newSpeed;
}

bool Simulation::paused() const { return paused_; }

void Simulation::togglePause() {
  if (paused_) {
    synchronize();
  }
  paused_ = !paused_;
}

void Simulation::addForce(const Force& force) { forces_.emplace_back(&force); }

void Simulation::addDamping(const Damping& damping) { dampings_.emplace_back(&damping); }

void Simulation::synchronize() { synchronizer_.synchronize(ps_.time); }