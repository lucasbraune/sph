#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>
#include <chrono>
#include <memory>
#include <functional>
#include "particle_system.hpp"

using std::vector;
using std::chrono::high_resolution_clock;
using std::chrono::time_point;
using std::unique_ptr;
using std::function;

struct SimulationInterface {
  virtual ~SimulationInterface() {}

  virtual const vector<Vec2d>& positions() const = 0;
  virtual double time() const = 0;
  
  virtual void computeNextFrame() = 0;
  virtual void waitForNextFrame() = 0;

  virtual double targetSpeed() const = 0;
  virtual void increaseTargetSpeed() = 0;
  virtual void decreaseTargetSpeed() = 0;

  virtual bool paused() const = 0;
  virtual void togglePause() = 0;
};

class Synchronizer {
public:
  Synchronizer();
  void synchronize(const double simulationTime);
  void waitUntil(const double simulationTime, const double simulationSpeed) const;

private:
  // Time of an event, as recorded by the system clock
  time_point<high_resolution_clock> fixedRealTime_;
  // Time of the same event, as recorded by the simulation clock. Measured in seconds.
  double fixedSimTime_;
};

template<class PhysicsType>
class Simulation : public SimulationInterface {
public:
  Simulation(const ParticleSystem& ps,
             const PhysicsType& physics,
             unique_ptr<TimeIntegrator> integrator = std::make_unique<VerletIntegrator>(0.01),
             double simulationSpeed = 1.0, int fps = 60);
  
  const vector<Vec2d>& positions() const;
  double time() const;
  
  void computeNextFrame();
  void waitForNextFrame();

  double targetSpeed() const;
  void increaseTargetSpeed();
  void decreaseTargetSpeed();

  bool paused() const;
  void togglePause();

private:
  void synchronize();
  void setTargetSpeed(double newSpeed);

  ParticleSystem ps_;
  PhysicsType physics_;
  unique_ptr<TimeIntegrator> integrator_;
  Synchronizer synchronizer_;
  double simulationSpeed_;
  int fps_;
  bool paused_;
};

template<class PhysicsType>
Simulation<PhysicsType>::Simulation(const ParticleSystem& ps, const PhysicsType& physics,
                       unique_ptr<TimeIntegrator> integrator,
                       double playbackSpeed, int fps)
    : ps_(ps),
      physics_(physics),
      integrator_(std::move(integrator)),
      synchronizer_(),
      simulationSpeed_(playbackSpeed),
      fps_(fps),
      paused_(true) {}

template<class PhysicsType>
const vector<Vec2d>& Simulation<PhysicsType>::positions() const { return ps_.positions; }

template<class PhysicsType>
double Simulation<PhysicsType>::time() const { return ps_.time; }

template<class PhysicsType>
void Simulation<PhysicsType>::computeNextFrame() {
  if (!paused()) {
    integrator_->integrate(ps_, physics_.forces(), physics_.dampings(), simulationSpeed_ / fps_);
  }
}

template<class PhysicsType>
void Simulation<PhysicsType>::waitForNextFrame() {
  if (!paused()) {
    synchronizer_.waitUntil(ps_.time, simulationSpeed_);
  }
}

template<class PhysicsType>
double Simulation<PhysicsType>::targetSpeed() const { return simulationSpeed_; }

template<class PhysicsType>
void Simulation<PhysicsType>::setTargetSpeed(double newSpeed) {
  synchronize();
  simulationSpeed_ = newSpeed;
}

template<class PhysicsType>
void Simulation<PhysicsType>::increaseTargetSpeed()
{
  setTargetSpeed(2.0 * targetSpeed());
}

template<class PhysicsType>
void Simulation<PhysicsType>::decreaseTargetSpeed()
{
  setTargetSpeed(0.5 * targetSpeed());
}

template<class PhysicsType>
bool Simulation<PhysicsType>::paused() const { return paused_; }

template<class PhysicsType>
void Simulation<PhysicsType>::togglePause() {
  if (paused_) {
    synchronize();
  }
  paused_ = !paused_;
}

template<class PhysicsType>
void Simulation<PhysicsType>::synchronize() { synchronizer_.synchronize(ps_.time); }

#endif