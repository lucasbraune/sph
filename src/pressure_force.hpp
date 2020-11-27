#ifndef PRESSURE_FORCE_H
#define PRESSURE_FORCE_H

#include <vector>
#include <memory>
#include <functional>
#include <cmath>
#include "util.hpp"

using std::vector;
using std::pair;
using std::unique_ptr;
using std::function;

class neighbor_iterator {
  public:
  virtual bool has_next() const = 0;
  virtual size_t next() = 0;
  virtual ~neighbor_iterator() {};
};

class neighbor_loop_util {
  public:
  virtual void refresh(const vector<Vec2d>& x) = 0; // when particles move
  virtual unique_ptr<neighbor_iterator> build(Vec2d x) const = 0;

  virtual unique_ptr<neighbor_loop_util> clone() const = 0;
  virtual ~neighbor_loop_util() {};
};

class smoothing_kernel {
  public:
  virtual double operator()(double dist) const = 0;
  virtual double differentated_at(double dist) const = 0;
  virtual double get_interaction_radius() const = 0;
  
  virtual unique_ptr<smoothing_kernel> clone() const = 0;
  virtual ~smoothing_kernel() {};
};

class pressure_force {
  public:
  pressure_force( unique_ptr<neighbor_loop_util> it_factory,
                  unique_ptr<smoothing_kernel> kernel,
                  function<double(double)> pressure_function);
  pressure_force(const pressure_force& sys); 
  pressure_force(pressure_force&& sys) = default;
  pressure_force& operator=(const pressure_force& sys);
  pressure_force& operator=(pressure_force&& sys) = default;
  ~pressure_force() = default;
  
  void apply(const double time, const double particleMass, const vector<Vec2d>& positions,
             vector<Vec2d>& accelerations) const;

  private:
  vector<double> compute_densities(double particleMass, const vector<Vec2d>& x) const;

  unique_ptr<smoothing_kernel> m_kernel;
  unique_ptr<neighbor_loop_util> m_loop_util;
  function<double(double)> m_pressure_function;
};

class cubic_kernel : public smoothing_kernel {
  public:
  cubic_kernel(double smoothing_length);
  double operator()(double dist) const override;
  double differentated_at(double dist) const override;
  double get_interaction_radius() const override;

  unique_ptr<smoothing_kernel> clone() const override;

  private:
  const double m_smoothing_length;
  const double m_C, m_D;
};

class water_pressure {
  public:
  water_pressure(double pressure_constant, double rest_density);
  double operator()(double density) const;

  private:
  const double m_pressure_constant;
  const double m_rest_density;
};

class gas_pressure {
  public:
  gas_pressure(double pressure_constant);
  double operator()(double density) const;

  private:
  const double m_pressure_constant;
};

#endif