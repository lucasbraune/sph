#include "pressure_force.hpp"

PressureForce::PressureForce(
    unique_ptr<NeighborIteratorFactory> iteratorFactory,
    unique_ptr<SmoothingKernel> kernel,
    function<double(double)> pressure) :
  kernel_(move(kernel)),
  neighborIteratorFactory_(move(iteratorFactory)),
  pressure_(pressure)
{}

PressureForce::PressureForce(const PressureForce& other) :
  kernel_(other.kernel_->clone()),
  neighborIteratorFactory_(other.neighborIteratorFactory_->clone()),
  pressure_(other.pressure_)
{}

PressureForce& PressureForce::operator=(const PressureForce& other)
{
  neighborIteratorFactory_ = other.neighborIteratorFactory_->clone();
  kernel_ = other.kernel_->clone();
  pressure_ = other.pressure_;
  return *this;

}

void PressureForce::apply(const double, const double particleMass,
                          const vector<Vec2d>& positions, vector<Vec2d>& accelerations) const
{
  neighborIteratorFactory_->refresh(positions);
  vector<double> densities = computeDensities(particleMass, positions);
  for (size_t i=0; i<positions.size(); i++) {
    Vec2d accPerMass = ZERO_VECTOR;
    double A = pressure_(densities[i]) / (densities[i] * densities[i]);
    auto it = neighborIteratorFactory_->build(positions[i]);
    while (it->hasNext()) {
      size_t j = it->next();
      if (j == i) continue; // OK to skip, assuming (1/r)(dW/dr) -> 0 as r->0
      double B = pressure_(densities[i]) / (densities[j] * densities[j]);
      double distance = dist(positions[i], positions[j]);
      double C = (A + B) * kernel_->DifferentiatedAt(distance) / distance;
      accPerMass -= C * (positions[i] - positions[j]);
    }
    accelerations[i] += particleMass * accPerMass;
  }
}

vector<double> PressureForce::computeDensities(double particleMass, const vector<Vec2d>& positions) const
{
  vector<double> densities(positions.size());
  for (size_t i=0; i<positions.size(); i++) {
    densities[i] = 0;
    auto it = neighborIteratorFactory_->build(positions[i]);
    while (it->hasNext()) {
      densities[i] += (*kernel_)(dist(positions[i], positions[it->next()]));
    }
    densities[i] *= particleMass;
  }
  return densities;
}

template<typename T>
RangeIterator<T>::RangeIterator(T begin, T end) :
  begin_(begin),
  end_(end),
  next_(begin)
{}

template<typename T>
bool RangeIterator<T>::hasNext() const 
{
  return next_ < end_;
}

template<typename T>
T RangeIterator<T>::next()
{
  return next_++; // return pre-increment value
}

TrivialNeighborIteratorFactory::TrivialNeighborIteratorFactory(size_t numberOfParticles) :
  numberOfParticles_(numberOfParticles)
{}

void TrivialNeighborIteratorFactory::refresh(const vector<Vec2d>& positions)
{
  numberOfParticles_ = positions.size();
}

unique_ptr<NeighborIterator> TrivialNeighborIteratorFactory::build(Vec2d) const
{
  return std::make_unique<RangeIterator<size_t>>(0, numberOfParticles_);
}

unique_ptr<NeighborIteratorFactory> TrivialNeighborIteratorFactory::clone() const
{
  return std::make_unique<TrivialNeighborIteratorFactory>(numberOfParticles_);
}

CubicKernel::CubicKernel(double smoothingLength) :
  smoothingLength_(smoothingLength),
  C_(15.0/ (14 * M_PI * smoothingLength * smoothingLength)),
  D_(-3 * C_ / smoothingLength_)
{}

double CubicKernel::operator()(double dist) const
{
  assert(dist >= 0);
  double q = dist/smoothingLength_;
  double A = 2 - q;
  if (q < 1) {
    double B = 1 - q;
    return C_ * (A*A*A - 4 * B*B*B);
  } else if (q < 2) {
    return C_ * A*A*A;
  } else {
    return 0.0;
  }
}

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

WaterPressure::WaterPressure(double pressureConstant, double restDensity) :
  pressureConstant_(pressureConstant),
  restDensity_(restDensity)
{}

double WaterPressure::operator()(double density) const
{
  return pressureConstant_ * (pow((density / restDensity_), 7) - 1);
}

GasPressure::GasPressure(double pressureConstant) :
  pressureConstant_(pressureConstant)
{}

double GasPressure::operator()(double density) const
{
  return pressureConstant_ * density * density;
}
