#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>
#include <chrono>
#include "particle_system.hpp"

using std::vector;
using std::chrono::high_resolution_clock;
using std::chrono::time_point;

class TimeUtil {
public:
  TimeUtil();
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
  Simulation(ParticleSystem& ps, const double simulationSpeed = 1.0, const int fps = 60);
  
  void computeNextFrame();
  void waitForNextFrame() const;

  double targetSpeed() const;
  void speedUp();
  void speedDown();

  bool paused() const;
  void switchPauseState();
  
  const vector<Vec2d>& positions() const;
  double time() const;

private:
  void synchronize();

  ParticleSystem& ps_;
  TimeUtil timeUtil_;
  double simulationSpeed_;
  int fps_;
  bool paused_;
};


#endif