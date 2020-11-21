#ifndef SYNCHRONIZER_HPP
#define SYNCHRONIZER_HPP

#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::time_point;

class Synchronizer {
public:
  Synchronizer(int targetFps, double playbackSpeed);
  Synchronizer(int targetFps);

  void setStart();
  void waitUntil(double simulationTime) const;
  double timeUntilNextFrame() const;

private:
  time_point<high_resolution_clock> sim_start_;
  const int fps_;
  const double playbackSpeed_;  
};


#endif