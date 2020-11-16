#include "particle_system.hpp"

ParticleSystem::ParticleSystem(const vector<Vec2d>& positions) :
  positions_(positions) {}

const vector<Vec2d>& ParticleSystem::positions() const
{
  return positions_;
}