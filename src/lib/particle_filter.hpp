#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <vector>
#include "particle_system.hpp"
#include "util.hpp"
#include "range/v3/view/any_view.hpp"
#include "range/v3/view/iota.hpp"

namespace sph {

struct ParticleFilter {
  virtual ~ParticleFilter() {};
  virtual std::unique_ptr<ParticleFilter> clone() const = 0;
  virtual void syncWith(const ParticleSystem& ps) = 0;
  virtual ranges::any_view<const Particle&> particlesIn(const Disk& neighborhood) const = 0; 
};

class TrivialFilter final : public ParticleFilter {
public:
  void syncWith(const ParticleSystem& ps) { ps_ = &ps; };
  std::unique_ptr<ParticleFilter> clone() const;
  ranges::any_view<const Particle&> particlesIn(const Disk&) const final;

private:
  const ParticleSystem* ps_;
};

namespace detail {

struct SubdividedInterval {
  SubdividedInterval(double left, double right, size_t subdivisions);
  size_t subdivision(double x) const;
  ranges::closed_iota_view<size_t, size_t> subdivisionRange(double xmin, double xmax) const;

  const double left;
  const double right;
  const size_t subdivisions;
};

} // namespace detail

class GridFilter final : public ParticleFilter {
public:
  GridFilter(const Rectangle& rect, size_t rows, size_t cols);
  std::unique_ptr<ParticleFilter> clone() const;
  void syncWith(const ParticleSystem& ps) final;
  ranges::any_view<const Particle&> particlesIn(const Disk& neighborhood) const final;

private:
  size_t row(const Vec2d& v) const { return width_.subdivision(v[0]); }
  size_t column(const Vec2d& v) const { return height_.subdivision(v[1]); }
  ranges::closed_iota_view<size_t, size_t> rowRange(const Disk& neighborhood) const;
  ranges::closed_iota_view<size_t, size_t> columnRange(const Disk& neighborhood) const;

  Matrix<std::vector<const Particle*>> grid_;
  const detail::SubdividedInterval width_, height_;
};

GridFilter makeGridFilter(const Rectangle& rect, double minimumCellLength);

} // namespace sph

#endif