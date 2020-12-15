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

class SimulationInterface {
public:
  virtual ~SimulationInterface() {}

  virtual const vector<Vec2d>& positions() const = 0;
  virtual double time() const = 0;
  virtual double targetSpeed() const = 0;
  virtual bool paused() const = 0;
  virtual void computeNextFrame() = 0;
  virtual void waitForNextFrame() = 0;
  virtual void togglePause() = 0;

  void increaseTargetSpeed() { setTargetSpeed(2.0 * targetSpeed()); }
  void decreaseTargetSpeed() { setTargetSpeed(0.5 * targetSpeed()); }

private:
  virtual void setTargetSpeed(double newSpeed) = 0;
};

class Synchronizer {
public:
  Synchronizer();
  void synchronize(double simulationTime);
  void waitUntil(double simulationTime, double simulationSpeed) const;

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
             double simulationSpeed = 1.0, int fps = 60) :
    ps_{ps},
    physics_{physics},
    integrator_{std::move(integrator)},
    synchronizer_{},
    simulationSpeed_{simulationSpeed},
    fps_{fps},
    paused_{true} {}
  
  const vector<Vec2d>& positions() const { return ps_.positions; }
  double time() const { return ps_.time; }
  double targetSpeed() const { return simulationSpeed_; }
  bool paused() const { return paused_; }
  
  void computeNextFrame()
  {
    if (!paused()) {
      integrator_->integrate(ps_, physics_.createForceVector(), physics_.createDampingVector(),
                             simulationSpeed_ / fps_);
    }
  }

  void waitForNextFrame()
  {
    if (!paused()) {
      synchronizer_.waitUntil(ps_.time, simulationSpeed_);
    }
  }

  void togglePause()
  {
    if (paused_) {
      synchronize();
    }
    paused_ = !paused_;
  }

private:
  void setTargetSpeed(double newSpeed)
  {
    synchronize();
    simulationSpeed_ = newSpeed;
  }

  void synchronize() { synchronizer_.synchronize(ps_.time); }

  ParticleSystem ps_;
  PhysicsType physics_;
  unique_ptr<TimeIntegrator> integrator_;
  Synchronizer synchronizer_;
  double simulationSpeed_;
  const int fps_;
  bool paused_;
};

#endif