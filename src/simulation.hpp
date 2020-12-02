#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>
#include <chrono>
#include <memory>
#include "particle_system.hpp"

using std::vector;
using std::chrono::high_resolution_clock;
using std::chrono::time_point;
using std::unique_ptr;

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

class SimulationRunner {
public:
  SimulationRunner(ParticleSystem& ps,
                   unique_ptr<TimeIntegrator> integrator = std::make_unique<VerletIntegrator>(0.01),
                   double simulationSpeed = 1.0, int fps = 60);
  
  const vector<Vec2d>& positions() const;
  void computeNextFrame();
  void waitForNextFrame();

  double targetSpeed() const;
  void speedUp();
  void speedDown();

  bool paused() const;
  void pauseOrUnpause();

private:
  void synchronize();

  ParticleSystem& ps_;
  unique_ptr<TimeIntegrator> integrator_;
  Synchronizer synchronizer_;
  double simulationSpeed_;
  int fps_;
  bool paused_;
};

class Simulation {
public:
  virtual ~Simulation() {};
  virtual SimulationRunner& runner() = 0;
  virtual const SimulationRunner& runner() const = 0;
  virtual const ParticleSystem& state() const = 0;
};


#endif