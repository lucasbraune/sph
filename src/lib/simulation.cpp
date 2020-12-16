#include "simulation.hpp"
#include <thread>

using std::chrono::milliseconds;

static milliseconds toMilliseconds(double seconds) {
  return milliseconds((long)(seconds * 1e3));
}

namespace sph {

Synchronizer::Synchronizer()
    : fixedRealTime_(high_resolution_clock::now()), fixedSimTime_(0.0) {}

void Synchronizer::synchronize(const double simulationTime) {
  fixedRealTime_ = high_resolution_clock::now();
  fixedSimTime_ = simulationTime;
}

void Synchronizer::waitUntil(const double simulationTime,
                             const double playbackSpeed) const {
  double difference = simulationTime - fixedSimTime_;
  auto realTime = fixedRealTime_ + toMilliseconds(difference / playbackSpeed);
  std::this_thread::sleep_until(realTime);
}

} // end namespace sph

