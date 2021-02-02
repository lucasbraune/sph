#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <chrono>
#include <type_traits>
#include "particle_system.hpp"

namespace sph {

namespace detail {

class Synchronizer {
public:
  Synchronizer();
  void synchronize(double simulationNow);
  void waitUntil(double simulationTime, double simulationSpeed) const;

private:
  // Time of an event, as recorded by the system clock
  std::chrono::time_point<std::chrono::high_resolution_clock> fixedRealTime_;
  // Time of the same event, as recorded by the simulation clock; in seconds
  double fixedSimTime_;
};

} // end namespace detail

struct SimulationState {
  const ParticleSystem& ps;
  const double targetSpeed;
  const bool paused;
};

template<class PhysicsType, class IntegratorType = Verlet>
class Simulation {
  static_assert(std::is_base_of_v<Physics, PhysicsType>);
  static_assert(std::is_base_of_v<TimeIntegrator, IntegratorType>);
  
public:
  Simulation(const ParticleSystem& ps,
             const PhysicsType& physics,
             const IntegratorType& integrator,
             double simulationSpeed = 1.0, int fps = 60) :
    physics_{physics},
    ps_{ps},
    integrator_{integrator},
    synchronizer_{},
    targetSpeed_{simulationSpeed},
    fps_{fps},
    paused_{true} {}
  
  virtual ~Simulation() {}
  
  SimulationState state() const { return {ps_, targetSpeed_, paused_}; }

  void computeNextFrame() 
  {
    if (!paused_) integrator_.integrate(ps_, physics_, targetSpeed_ / fps_);
  }

  void waitForNextFrame()
  {
    if (!paused_) synchronizer_.waitUntil(ps_.time, targetSpeed_);
  }

  void togglePause()
  {
    if (paused_) synchronize();
    paused_ = !paused_;
  }

  void increaseTargetSpeed() { setTargetSpeed(2.0 * targetSpeed_); }
  void decreaseTargetSpeed() { setTargetSpeed(0.5 * targetSpeed_); }

protected:
  PhysicsType physics_;

private:
  void synchronize() { synchronizer_.synchronize(ps_.time); }
  void setTargetSpeed(double newSpeed)
  {
    synchronize();
    targetSpeed_ = newSpeed;
  }
  
  ParticleSystem ps_;
  IntegratorType integrator_;
  detail::Synchronizer synchronizer_;
  double targetSpeed_;
  const int fps_;
  bool paused_;
};

} // end namespace sph

#endif