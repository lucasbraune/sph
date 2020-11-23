#include "central_gravity.hpp"

ParticleSystem createParticleSystem(const size_t numberOfParticles,
    const double totalMass, const Rectangle region, const double timeStep)
{
  return ParticleSystem{
      randomVectors(region, numberOfParticles),
      vector<Vec2d>(numberOfParticles),
      vector<Force*>{},
      NULL,
      totalMass/numberOfParticles,
      timeStep};
}

CentralGravity::CentralGravity(const size_t numberOfParticles, const double totalMass,
    const Rectangle region, const double gravityConstant, const double dampingConstant,
    const double timeStep) :
  Simulation(createParticleSystem(numberOfParticles, totalMass, region, timeStep)),
  gravity_{ZERO_VECTOR, gravityConstant},
  damping_{dampingConstant}
{
  addForce(&gravity_);
  replaceDamping(&damping_);
}