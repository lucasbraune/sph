#include "demo.hpp"
#include <functional>
#include <cmath>

using std::reference_wrapper;

AdjustableParameter::AdjustableParameter(const function<double(void)>& getter, const function<void(double)>& setter) :
  get_(getter),
  set_(setter)
{}

AdjustableParameter::AdjustableParameter(PointGravity& gravity) :
  get_([&gravity](){ return gravity.constant(); }),
  set_([&gravity](double x){ gravity.setConstant(x); })
{}

double AdjustableParameter::value() const
{
  return get_();
}

void AdjustableParameter::increase()
{
  set_(2.0 * get_());
}

void AdjustableParameter::decrease()
{
  set_(0.5 * get_());
}

CentralPotential::CentralPotential(size_t numberOfParticles, double totalMass, Rectangle region,
                                   double gravityConstant, double dampingConstant) :
  gravity_(gravityConstant),
  damping_(dampingConstant),
  ps_(numberOfParticles, totalMass, region, {std::cref<Force>(gravity_)}, damping_),
  runner_(ps_),
  gravityConstant_(gravity_)
{}

SimulationRunner& CentralPotential::runner()
{
  return runner_;
}

const SimulationRunner& CentralPotential::runner() const
{
  return runner_;
}

LinearDamping& CentralPotential::damping()
{
  return damping_;
}

AdjustableParameter& CentralPotential::gravity()
{
  return gravityConstant_;
}

const AdjustableParameter& CentralPotential::gravity() const
{
  return gravityConstant_;
}

static double gravityConstant(double totalMass, double pressureConstant, double starRadius)
{
  return 8 * totalMass * pressureConstant / (M_PI * pow(starRadius, 4));
}

static double interactionRadius(double numberOfParticles)
{
  return sqrt(10.0 / numberOfParticles);
}

ToyStar::ToyStar(size_t numberOfParticles, double totalMass, double starRadius, Rectangle region,
                 double dampingConstant, double pressureConstant) :
  gravity_(gravityConstant(totalMass, pressureConstant, starRadius)),
  pressureForce_(interactionRadius(numberOfParticles), GasPressure(pressureConstant)),
  damping_(dampingConstant),
  ps_(numberOfParticles, totalMass, region,
      {std::cref<Force>(gravity_), std::cref<Force>(pressureForce_)}, damping_),
  runner_(ps_),
  gravityConstant_(gravity_)
{}

SimulationRunner& ToyStar::runner()
{
  return runner_;
}

const SimulationRunner& ToyStar::runner() const
{
  return runner_;
}

LinearDamping& ToyStar::damping()
{
  return damping_;
}

AdjustableParameter& ToyStar::gravity()
{
  return gravityConstant_;
}

const AdjustableParameter& ToyStar::gravity() const
{
  return gravityConstant_;
}