#include "particle_system.hpp"

using namespace sph;

sph::ParticleSystem::ParticleSystem(const std::vector<Particle>& particles, double totalMass) :
  particles{particles},
  particleMass{totalMass / particles.size()}
{
  assert(particles.size() > 0);
}

std::vector<Particle> sph::randomParticles(const Rectangle& region, size_t numberOfParticles)
{
  std::vector<Particle> result;
  for (size_t i = 0; i < numberOfParticles; ++i) {
    auto p = Particle{randomVec2d(region)};
    result.emplace_back(p);
  }
  return result;
}

sph::PointGravity::PointGravity(double gravityConstant, const Vec2d& center) :
  center_(center), intensity_(gravityConstant)
{}

void sph::PointGravity::apply(ParticleSystem& ps)
{
  for (auto& particle : ps.particles) {
    particle.acc -= intensity_ * (particle.pos - center_);
  }
}

sph::SurfaceGravity::SurfaceGravity(double magnitude) :
  acceleration_{0.0, -magnitude} {}

void sph::SurfaceGravity::apply(ParticleSystem& ps)
{
  for (auto& particle : ps.particles) {
    particle.acc += acceleration_;
  }
}

sph::LinearDamping::LinearDamping(double dampingConstant) :
  intensity_(dampingConstant)
{}

void sph::LinearDamping::apply(ParticleSystem& ps) const
{
  for (auto& particle : ps.particles) {
    particle.acc -= (intensity_ / ps.particleMass) * particle.vel;
  }
}

sph::Wall::Wall(const Vec2d& normal, double distanceFromTheOrigin) :
  unitNormal_{unit(normal)},
  ptOnWall_{-distanceFromTheOrigin * unitNormal_} {}

void sph::Wall::resolveCollisions(ParticleSystem& ps) const
{
  for (auto& particle : ps.particles) {
    resolveCollision(particle.pos, particle.vel);
  }
}

void sph::Wall::resolveCollision(Vec2d& pos, Vec2d& vel) const
{
  auto w = pos - ptOnWall_;
  if (dotProduct(w, unitNormal_) > 0) return;
  pos -= 2 * project(w, unitNormal_);
  vel -= 2 * project(vel, unitNormal_);
}