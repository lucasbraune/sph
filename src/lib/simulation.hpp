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

class Simulation {
public:
  Simulation(const ParticleSystem& ps,
             const vector<const Force*>& forces,
             const vector<const Damping*>& dampings,
             unique_ptr<TimeIntegrator> integrator = std::make_unique<VerletIntegrator>(0.01),
             double simulationSpeed = 1.0, int fps = 60);
  Simulation(size_t numberOfParticles, double totalMass, Rectangle region);
  
  const vector<Vec2d>& positions() const;
  double time() const;
  
  void computeNextFrame();
  void waitForNextFrame();

  double targetSpeed() const;
  void increaseTargetSpeed();
  void decreaseTargetSpeed();

  bool paused() const;
  void togglePause();

protected:
  void addForce(const Force& force);
  void addDamping(const Damping& damping);

private:
  void synchronize();
  void setTargetSpeed(double newSpeed);

  ParticleSystem ps_;
  vector<const Force*> forces_;
  vector<const Damping*> dampings_;
  unique_ptr<TimeIntegrator> integrator_;
  Synchronizer synchronizer_;
  double simulationSpeed_;
  int fps_;
  bool paused_;
};


#endif