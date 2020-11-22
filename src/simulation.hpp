#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>
#include "particle_system.hpp"
#include "time_controller.hpp"

using std::vector;

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