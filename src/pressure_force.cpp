#include "pressure_force.hpp"
#include <cassert>

pressure_force::pressure_force(
    unique_ptr<neighbor_loop_util> it_factory,
    unique_ptr<smoothing_kernel> kernel,
    function<double(double)> pressure_function) :
  m_kernel(move(kernel)),
  m_loop_util(move(it_factory)),
  m_pressure_function(pressure_function)
{}

pressure_force::pressure_force(const pressure_force& sys) :
  m_kernel(sys.m_kernel->clone()),
  m_loop_util(sys.m_loop_util->clone()),
  m_pressure_function(sys.m_pressure_function)
{}

pressure_force& pressure_force::operator=(const pressure_force& sys)
{
  m_loop_util = sys.m_loop_util->clone();
  m_kernel = sys.m_kernel->clone();
  m_pressure_function = sys.m_pressure_function;
  return *this;

}

void pressure_force::apply(const double, const double particleMass,
                           const vector<Vec2d>& x, vector<Vec2d>& a) const
{
  m_loop_util->refresh(x);
  vector<double> densities = compute_densities(particleMass, x);
  for (size_t i=0; i<x.size(); i++) {
    Vec2d ai = ZERO_VECTOR;
    double A = m_pressure_function(densities[i]) / (densities[i] * densities[i]);
    auto& it = *(m_loop_util->build(x[i]));
    while (it.has_next()) {
      size_t j = it.next();
      if (j == i) continue; // OK to skip, assuming (1/r)(dW/dr) -> 0 as r->0
      double B = m_pressure_function(densities[i]) / (densities[j] * densities[j]);
      double d = dist(x[i], x[j]);
      double C = (A + B) * m_kernel->differentated_at(d) / d;
      ai -= C * (x[i] - x[j]);
    }
    a[i] += particleMass * ai;
  }
}

vector<double> pressure_force::compute_densities(double particleMass, const vector<Vec2d>& x) const
{
  vector<double> densities(x.size());
  for (size_t i=0; i<x.size(); i++) {
    densities[i] = 0;
    auto& it = *(m_loop_util->build(x[i]));
    while (it.has_next()) {
      densities[i] += (*m_kernel)(dist(x[i], x[it.next()]));
    }
    densities[i] *= particleMass;
  }
  return densities;
}

cubic_kernel::cubic_kernel(double smoothing_length) :
  m_smoothing_length(smoothing_length),
  m_C(15.0/ (14 * M_PI * smoothing_length * smoothing_length)),
  m_D(-3 * m_C / m_smoothing_length)
{}

double cubic_kernel::operator()(double dist) const
{
  if (dist < 0 || dist > 2) return 0;
  double q = dist/m_smoothing_length;
  double A = 2 - q;
  if (dist < 1) {
    double B = 1 - q;
    return m_C * (A*A*A - 4 * B*B*B);
  } else {
    // 1 <= dist <= 2
    return m_C * A*A*A;
  }
}

// Derivative of kernel as a function of one variable (dist)
double cubic_kernel::differentated_at(double dist) const
{
  if (dist < 0 || dist > 2) return 0;
  double q = dist/m_smoothing_length;
  double A = 2 - q;
  if (dist < 1) {
    double B = 1 - q;
    return m_D * (A*A - 4 * B*B);
  } else {
    // 1 <= dist <= 2
    return m_D * A*A;
  }
}

double cubic_kernel::get_interaction_radius() const 
{
  return m_smoothing_length * 2;
}

unique_ptr<smoothing_kernel> cubic_kernel::clone() const
{
  return std::make_unique<cubic_kernel>(*this);
}

water_pressure::water_pressure(
    double pressure_constant, double rest_density) :
  m_pressure_constant(pressure_constant),
  m_rest_density(rest_density)
{}

double water_pressure::operator()(double density) const
{
  return m_pressure_constant * (pow((density / m_rest_density), 7) - 1);
}

gas_pressure::gas_pressure(double pressure_constant) :
  m_pressure_constant(pressure_constant)
{}

double gas_pressure::operator()(double density) const
{
  return m_pressure_constant * density * density;
}
