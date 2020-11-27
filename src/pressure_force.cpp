#include "pressure_force.hpp"

PressureForce::PressureForce(
    unique_ptr<NeighborIteratorFactory> it_factory,
    unique_ptr<SmoothingKernel> kernel,
    function<double(double)> pressure_function) :
  kernel_(move(kernel)),
  neighborIteratorFactory_(move(it_factory)),
  pressure_(pressure_function)
{}

PressureForce::PressureForce(const PressureForce& sys) :
  kernel_(sys.kernel_->clone()),
  neighborIteratorFactory_(sys.neighborIteratorFactory_->clone()),
  pressure_(sys.pressure_)
{}

PressureForce& PressureForce::operator=(const PressureForce& sys)
{
  neighborIteratorFactory_ = sys.neighborIteratorFactory_->clone();
  kernel_ = sys.kernel_->clone();
  pressure_ = sys.pressure_;
  return *this;

}

void PressureForce::apply(const double, const double particleMass,
                           const vector<Vec2d>& x, vector<Vec2d>& a) const
{
  neighborIteratorFactory_->refresh(x);
  vector<double> densities = computeDensities(particleMass, x);
  for (size_t i=0; i<x.size(); i++) {
    Vec2d accPerMass = ZERO_VECTOR;
    double A = pressure_(densities[i]) / (densities[i] * densities[i]);
    auto& it = *(neighborIteratorFactory_->build(x[i]));
    while (it.hasNext()) {
      size_t j = it.next();
      if (j == i) continue; // OK to skip, assuming (1/r)(dW/dr) -> 0 as r->0
      double B = pressure_(densities[i]) / (densities[j] * densities[j]);
      double distance = dist(x[i], x[j]);
      double C = (A + B) * kernel_->DifferentiatedAt(distance) / distance;
      accPerMass -= C * (x[i] - x[j]);
    }
    a[i] += particleMass * accPerMass;
  }
}

vector<double> PressureForce::computeDensities(double particleMass, const vector<Vec2d>& x) const
{
  vector<double> densities(x.size());
  for (size_t i=0; i<x.size(); i++) {
    densities[i] = 0;
    auto& it = *(neighborIteratorFactory_->build(x[i]));
    while (it.hasNext()) {
      densities[i] += (*kernel_)(dist(x[i], x[it.next()]));
    }
    densities[i] *= particleMass;
  }
  return densities;
}

CubicKernel::CubicKernel(double smoothing_length) :
  smoothingLength_(smoothing_length),
  C_(15.0/ (14 * M_PI * smoothing_length * smoothing_length)),
  D_(-3 * C_ / smoothingLength_)
{}

double CubicKernel::operator()(double dist) const
{
  if (dist < 0 || dist > 2) return 0;
  double q = dist/smoothingLength_;
  double A = 2 - q;
  if (dist < 1) {
    double B = 1 - q;
    return C_ * (A*A*A - 4 * B*B*B);
  } else {
    // 1 <= dist <= 2
    return C_ * A*A*A;
  }
}

// Derivative of kernel as a function of one variable (dist)
double CubicKernel::DifferentiatedAt(double dist) const
{
  if (dist < 0 || dist > 2) return 0;
  double q = dist/smoothingLength_;
  double A = 2 - q;
  if (dist < 1) {
    double B = 1 - q;
    return D_ * (A*A - 4 * B*B);
  } else {
    // 1 <= dist <= 2
    return D_ * A*A;
  }
}

double CubicKernel::interactionRadius() const 
{
  return smoothingLength_ * 2;
}

unique_ptr<SmoothingKernel> CubicKernel::clone() const
{
  return std::make_unique<CubicKernel>(*this);
}

WaterPressure::WaterPressure(
    double pressure_constant, double rest_density) :
  pressureConstant_(pressure_constant),
  restDensity_(rest_density)
{}

double WaterPressure::operator()(double density) const
{
  return pressureConstant_ * (pow((density / restDensity_), 7) - 1);
}

GasPressure::GasPressure(double pressure_constant) :
  pressureConstant_(pressure_constant)
{}

double GasPressure::operator()(double density) const
{
  return pressureConstant_ * density * density;
}
