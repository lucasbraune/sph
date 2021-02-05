/**
 * File: summation_strategy.hpp
 * 
 * This header declares types that represent strategies for performing sums over
 * the particles of a particle system.
 */

#ifndef SUMMATION_STRATEGY_HPP
#define SUMMATION_STRATEGY_HPP

#include <cassert>
#include <type_traits>
#include "particle_system.hpp"

namespace sph {

class TrivialSummation final {
public:
  /**
   * Synchronizes this object with the specified particle system.
   */
  void syncWith(const ParticleSystem& ps) { ps_ = &ps; };
  
  /**
   * Returns the sum of the values of a function evaluated at all particles of a
   * particle system.
   * 
   * Precondition: This object must be synchronized with the specified particle system.
   */
  template<class SummandFn>
  std::result_of_t<SummandFn(const Particle&)>
  sumOverParticles(SummandFn&& summand, const Disk&, const ParticleSystem& ps) const
  {
    assert(inSync(ps));
    auto sum = std::result_of_t<SummandFn(const Particle&)>{};
    for (auto& particle : ps_->particles) {
      sum += summand(particle);
    }
    return sum;
  }

private:
  bool inSync(const ParticleSystem& ps) const { return ps_ == &ps; }
  const ParticleSystem* ps_;
};

namespace detail {

/**
 * Represents a half-open interval [left, right) subdivided into half-open subintervals of
 * equal length.
 */
struct SubdividedInterval final {
  // Preconditions: left < right, subdivisions > 0
  SubdividedInterval(double left, double right, size_t subdivisions);

  /**
   * Returns the index of the subdivision containing a specified real number x.
   * If there is no such subdivision, returns 0 or the number of subdivisions minus one,
   * according to whether x < left or x >= right. 
   */
  size_t subdivision(double x) const;

  const double left;
  const double right;
  const size_t subdivisions;
};

} // namespace detail

class GridBasedSummation final {
public:
  // Precondition: rows > 0, cols > 0
  GridBasedSummation(const Rectangle& rect, size_t rows, size_t cols);
  // Precondition: minimumCellLength > 0
  GridBasedSummation(const Rectangle& rect, double minimumCellLength);

  /**
   * Synchronizes this object with the specified particle system. Synchronization lasts
   * until the positions of the particles in the specified particle system change.
   */
  void syncWith(const ParticleSystem& ps);
  
  /**
   * Returns the sum of the values of a function evaluated at all particles of a
   * particle system. The second argument is a disk outside of which the function
   * is identically zero.
   * 
   * Precondition: This object must be synchronized with the specified particle system.
   */
  template<class SummandFn>
  std::result_of_t<SummandFn(const Particle&)>
  sumOverParticles(SummandFn&& summand, const Disk& support, const ParticleSystem& ps) const
  {
    assert(inSync(ps));

    const auto minI = row(support.center[0] - support.radius);
    const auto maxI = row(support.center[0] + support.radius);
    const auto minJ = column(support.center[1] - support.radius);
    const auto maxJ = column(support.center[1] + support.radius);
    
    auto sum = std::result_of_t<SummandFn(const Particle&)>{};
    for (size_t i=minI; i<=maxI; ++i) {
      for (size_t j=minJ; j<=maxJ; ++j) {
        for (auto* particlePtr : grid_(i, j)) {
          sum += summand(*particlePtr);
        }
      }
    }
    return sum;
  }

private:
  size_t row(double x) const { return width_.subdivision(x); }
  size_t column(double y) const { return height_.subdivision(y); }
  bool inSync(const ParticleSystem& ps) const
  {
    return ps_ == &ps && lastSynchronized_ == ps.time;
  }
  
  Matrix<std::vector<const Particle*>> grid_;
  const detail::SubdividedInterval width_, height_;
  const ParticleSystem* ps_;
  double lastSynchronized_;
};

} // namespace sph

#endif