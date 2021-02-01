#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <chrono>
#include <type_traits>
#include "particle_system.hpp"
#include "range/v3/view/any_view.hpp"

namespace sph {

class SimulationInterface {
public:
  virtual ~SimulationInterface() {}

  virtual ranges::any_view<const Vec2d&> positions() const = 0;
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
  void synchronize(double simulationNow);
  void waitUntil(double simulationTime, double simulationSpeed) const;

private:
  // Time of an event, as recorded by the system clock
  std::chrono::time_point<std::chrono::high_resolution_clock> fixedRealTime_;
  // Time of the same event, as recorded by the simulation clock; in seconds
  double fixedSimTime_;
};

} // end namespace detail

template<class PhysicsType, class IntegratorType = Verlet>
class Simulation : public SimulationInterface {
  static_assert(std::is_base_of_v<Physics, PhysicsType>);
  static_assert(std::is_base_of_v<TimeIntegrator, IntegratorType>);
  
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
  
  ranges::any_view<const Vec2d&> positions() const final { return sph::positions(ps_); }
  double time() const final { return ps_.time; }
  double targetSpeed() const final { return simulationSpeed_; }
  bool paused() const final { return paused_; }

  void computeNextFrame() final 
  {
    if (!paused()) integrator_.integrate(ps_, physics_, simulationSpeed_ / fps_);
  }

  void waitForNextFrame() final
  {
    if (!paused()) synchronizer_.waitUntil(ps_.time, simulationSpeed_);
  }

  void setTargetSpeed(double newSpeed) final
  {
    synchronize();
    simulationSpeed_ = newSpeed;
  }

  void togglePause() final
  {
    if (paused()) synchronize();
    paused_ = !paused_;
  }

private:
  void synchronize() { synchronizer_.synchronize(ps_.time); }
  ParticleSystem ps_;
  
protected:
  PhysicsType physics_;

private:
  IntegratorType integrator_;
  detail::Synchronizer synchronizer_;
  double simulationSpeed_;
  const int fps_;
  bool paused_;
};

} // end namespace sph

#endif