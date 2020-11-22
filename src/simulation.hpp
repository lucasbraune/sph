#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "particle_system.hpp"
#include "time_controller.hpp"

class Simulation {
public:
  Simulation(const ParticleSystem& ps, const TimeController& tc);
  void computeNextFrame();
  void waitForNextFrame() const;
  const ParticleSystem& particleSystem() const;

private:
  ParticleSystem ps_;
  TimeController tc_;
};

#endif