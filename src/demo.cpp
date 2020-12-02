#include "demo.hpp"
#include <cmath>

using std::reference_wrapper;


ParameterAdjuster::ParameterAdjuster(const function<double(void)>& getter, const function<void(double)>& setter) :
  get_(getter),
  set_(setter)
{}

void ParameterAdjuster::increase() const
{
  set_(2.0 * get_());
}

void ParameterAdjuster::decrease() const
{
  set_(0.5 * get_());
}

ParameterAdjuster AdjusterFactory::gravity(PointGravity& gravity)
{
  return ParameterAdjuster([&gravity](){ return gravity.constant(); },
                           [&gravity](double x){ gravity.setConstant(x); });
}

ParameterAdjuster AdjusterFactory::speed(SimulationRunner& runner)
{
  return ParameterAdjuster([&runner](){ return runner.targetSpeed(); },
                           [&runner](double x){ runner.setTargetSpeed(x); });
}

ParameterAdjuster AdjusterFactory::damping(LinearDamping& damping)
{
  return ParameterAdjuster([&damping](){ return damping.constant(); },
                           [&damping](double x){ damping.setConstant(x); });
}

CentralPotential::CentralPotential(size_t numberOfParticles, double totalMass, Rectangle region,
                                   double gravityConstant, double dampingConstant) :
  gravity_(gravityConstant),
  damping_(dampingConstant),
  ps_(numberOfParticles, totalMass, region, {std::cref<Force>(gravity_)}, damping_),
  runner_(ps_),
  speedAdjuster_(AdjusterFactory::speed(runner_)),
  dampingAdjuster_(AdjusterFactory::damping(damping_)),
  gravityAdjuster_(AdjusterFactory::gravity(gravity_))
{}

SimulationRunner& CentralPotential::runner()
{
  return runner_;
}

const SimulationRunner& CentralPotential::runner() const
{
  return runner_;
}

const ParameterAdjuster& CentralPotential::speedAdjuster()
{
  return speedAdjuster_;
}

const ParameterAdjuster& CentralPotential::dampingAdjuster()
{
  return dampingAdjuster_;
}

const ParameterAdjuster& CentralPotential::gravityAdjuster()
{
  return gravityAdjuster_;
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
  speedAdjuster_(AdjusterFactory::speed(runner_)),
  dampingAdjuster_(AdjusterFactory::damping(damping_)),
  gravityAdjuster_(AdjusterFactory::gravity(gravity_))
{}

SimulationRunner& ToyStar::runner()
{
  return runner_;
}

const SimulationRunner& ToyStar::runner() const
{
  return runner_;
}

const ParameterAdjuster& ToyStar::speedAdjuster()
{
  return speedAdjuster_;
}

const ParameterAdjuster& ToyStar::dampingAdjuster()
{
  return dampingAdjuster_;
}

const ParameterAdjuster& ToyStar::gravityAdjuster()
{
  return gravityAdjuster_;
}