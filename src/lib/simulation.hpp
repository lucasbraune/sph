#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <chrono>
#include <memory>
#include "particle_system.hpp"

namespace sph {

class SimulationInterface {
public:
  virtual ~SimulationInterface() {}

  virtual const std::vector<Vec2d>& positions() const = 0;
  virtual double time() const = 0;
  virtual double targetSpeed() const = 0;
  virtual bool paused() const = 0;
  virtual void computeNextFrame() = 0;
  virtual void waitForNextFrame() = 0;
  virtual void setTargetSpeed(double newSpeed) = 0;
  virtual void togglePause() = 0;

  void increaseTargetSpeed() { setTargetSpeed(2.0 * targetSpeed()); }
  void decreaseTargetSpeed() { setTargetSpeed(0.5 * targetSpeed()); }
};

namespace detail {

class Synchronizer {
public:
  Synchronizer();
  void synchronize(double simulationTime);
  void waitUntil(double simulationTime, double simulationSpeed) const;

private:
  // Time of an event, as recorded by the system clock
  std::chrono::time_point<
      std::chrono::high_resolution_clock> fixedRealTime_;
  // Time of the same event, as recorded by the simulation clock; in seconds
  double fixedSimTime_;
};

} // end namespace detail

template<class PhysicsType,                       // must be convertible to Physics&
         class IntegratorType = VerletIntegrator> // models implementation of TimeIntegrator
class Simulation : public SimulationInterface {
public:
  Simulation(const ParticleSystem& ps,
             const PhysicsType& physics,
             const IntegratorType& integrator,
             double simulationSpeed = 1.0, int fps = 60) :
    ps_{ps},
    physics_{physics},
    integrator_{integrator},
    synchronizer_{},
    simulationSpeed_{simulationSpeed},
    fps_{fps},
    paused_{true} {}
  
  const std::vector<Vec2d>& positions() const { return ps_.positions; }
  double time() const { return ps_.time; }
  double targetSpeed() const { return simulationSpeed_; }
  bool paused() const { return paused_; }
  
  void computeNextFrame()
  {
    if (!paused()) integrator_.integrate(ps_, physics_, simulationSpeed_ / fps_);
  }

  void waitForNextFrame()
  {
    if (!paused()) synchronizer_.waitUntil(ps_.time, simulationSpeed_);
  }

  void setTargetSpeed(double newSpeed)
  {
    synchronize();
    simulationSpeed_ = newSpeed;
  }

  void togglePause()
  {
    if (paused()) synchronize();
    paused_ = !paused_;
  }

private:
  void synchronize() { synchronizer_.synchronize(ps_.time); }

  ParticleSystem ps_;
  PhysicsType physics_;
  IntegratorType integrator_;
  detail::Synchronizer synchronizer_;
  double simulationSpeed_;
  const int fps_;
  bool paused_;
};

} // end namespace sph

#endif