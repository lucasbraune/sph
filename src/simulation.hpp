#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>
#include <chrono>
#include "particle_system.hpp"

using std::vector;
using std::chrono::high_resolution_clock;
using std::chrono::time_point;

class TimeController {
public:
  TimeController(int targetFps, double playbackSpeed);
  TimeController(int targetFps);

  void setStartOfSimulation();
  void waitUntil(double simulationTime) const;
  double timeUntilNextFrame() const;

private:
  time_point<high_resolution_clock> sim_start_;
  const int fps_;
  const double playbackSpeed_;  
};

class Simulation {
public:
  Simulation(const ParticleSystem& ps, const TimeController& tc);
  void computeNextFrame();
  void waitForNextFrame() const;

  const vector<Vec2d>& positions() const;
  double time() const;

private:
  ParticleSystem ps_;
  TimeController tc_;
};

#endif