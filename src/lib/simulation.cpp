#include "simulation.hpp"
#include <thread>

using std::chrono::milliseconds;
using std::chrono::high_resolution_clock;
using std::chrono::time_point;

static milliseconds toMilliseconds(double seconds) {
  return milliseconds((long)(seconds * 1e3));
}

namespace sph {
namespace detail {

Synchronizer::Synchronizer()
    : fixedRealTime_(high_resolution_clock::now()), fixedSimTime_(0.0) {}

void Synchronizer::synchronize(const double simulationNow) {
  fixedRealTime_ = high_resolution_clock::now();
  fixedSimTime_ = simulationNow;
}

void Synchronizer::waitUntil(const double simulationTime,
                             const double playbackSpeed) const {
  double difference = simulationTime - fixedSimTime_;
  auto realTime = fixedRealTime_ + toMilliseconds(difference / playbackSpeed);
  std::this_thread::sleep_until(realTime);
}

} // end namespace detail
} // end namespace sph

