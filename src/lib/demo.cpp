#include "demo.hpp"
#include <cmath>

using std::reference_wrapper;

CentralGravityPhysics::CentralGravityPhysics(double gravityConstant, double dampingConstant) :
  gravity_{gravityConstant},
  damping_{dampingConstant}
{
  forces_.emplace_back(&gravity_);
  dampings_.emplace_back(&damping_);
}

CentralGravityPhysics::CentralGravityPhysics(const CentralGravityPhysics& other) :
  gravity_{other.gravity_},
  damping_{other.damping_}
{
  forces_.emplace_back(&gravity_);
  dampings_.emplace_back(&damping_);
}

CentralGravityPhysics::CentralGravityPhysics(CentralGravityPhysics&& other) :
  gravity_{other.gravity_},
  damping_{other.damping_}
{
  forces_.emplace_back(&gravity_);
  dampings_.emplace_back(&damping_);
}

CentralGravityPhysics& CentralGravityPhysics::operator=(const CentralGravityPhysics& other)
{
  gravity_ = other.gravity_;
  damping_ = other.damping_;
  forces_ = {&gravity_};
  dampings_ = {&damping_};
  return *this;
}

CentralGravityPhysics& CentralGravityPhysics::operator=(CentralGravityPhysics&& other)
{
  gravity_ = other.gravity_;
  damping_ = other.damping_;
  forces_ = {&gravity_};
  dampings_ = {&damping_};
  return *this;
} 

ToyStarPhysics::ToyStarPhysics(double gravityConstant,
                               double dampingConstant,
                               double pressureConstant,
                               double interactionRadius) :
  gravity_{gravityConstant},
  damping_{dampingConstant},
  pressure_{interactionRadius, GasPressure{pressureConstant}}
{
  forces_.emplace_back(&gravity_);
  forces_.emplace_back(&pressure_);
  dampings_.emplace_back(&damping_);
}

ToyStarPhysics::ToyStarPhysics(const ToyStarPhysics& other) :
  gravity_{other.gravity_},
  damping_{other.damping_},
  pressure_{other.pressure_}
{
  forces_.emplace_back(&gravity_);
  forces_.emplace_back(&pressure_);
  dampings_.emplace_back(&damping_);
}

ToyStarPhysics::ToyStarPhysics(ToyStarPhysics&& other) :
  gravity_{other.gravity_},
  damping_{other.damping_},
  pressure_{other.pressure_}
{
  forces_.emplace_back(&gravity_);
  forces_.emplace_back(&pressure_);
  dampings_.emplace_back(&damping_);
}

ToyStarPhysics& ToyStarPhysics::operator=(const ToyStarPhysics& other)
{
  gravity_ = other.gravity_;
  damping_ = other.damping_;
  pressure_ = other.pressure_;
  forces_ = {&gravity_, &pressure_};
  dampings_ = {&damping_};
  return *this;
}

ToyStarPhysics& ToyStarPhysics::operator=(ToyStarPhysics&& other)
{
  gravity_ = other.gravity_;
  damping_ = other.damping_;
  pressure_ = other.pressure_;
  forces_ = {&gravity_, &pressure_};
  dampings_ = {&damping_};
  return *this;
} 

Simulation<CentralGravityPhysics> createCentralGravitySimulation(
    size_t numberOfParticles,
    double totalMass,
    Rectangle region,
    double gravityConstant,
    double dampingConstant)
{
  return {ParticleSystem{numberOfParticles, totalMass, region},
          CentralGravityPhysics{gravityConstant, dampingConstant}};
}

static double gravityConstant(double totalMass, double pressureConstant, double starRadius)
{
  return 8 * totalMass * pressureConstant / (M_PI * pow(starRadius, 4));
}

static double interactionRadius(double numberOfParticles)
{
  return sqrt(10.0 / numberOfParticles);
}

Simulation<ToyStarPhysics> createToyStarSimulation(
    size_t numberOfParticles,
    double starMass,
    double starRadius,
    Rectangle initialRegion,
    double dampingConstant,
    double pressureConstant)
{
  return {ParticleSystem{numberOfParticles, starMass, initialRegion},
          ToyStarPhysics{gravityConstant(starMass, pressureConstant, starRadius),
                         dampingConstant,
                         pressureConstant,
                         interactionRadius(numberOfParticles)}};
}