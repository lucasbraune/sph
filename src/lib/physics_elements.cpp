#include "physics_elements.hpp"

using std::vector;

namespace sph {

PointGravity::PointGravity(double gravityConstant, const Vec2d& center) :
  center_(center), intensity_(gravityConstant)
{}

void PointGravity::apply(ParticleSystem& ps)
{
  for (auto& particle : ps.particles) {
    particle.acc -= intensity_ * (particle.pos - center_);
  }
}

SurfaceGravity::SurfaceGravity(double magnitude) :
  acceleration_{0.0, -magnitude} {}

void SurfaceGravity::apply(ParticleSystem& ps)
{
  for (auto& particle : ps.particles) {
    particle.acc += acceleration_;
  }
}

LinearDamping::LinearDamping(double dampingConstant) :
  intensity_(dampingConstant)
{}

void LinearDamping::apply(ParticleSystem& ps) const
{
  for (auto& particle : ps.particles) {
    particle.acc -= (intensity_ / ps.particleMass) * particle.vel;
  }
}

Wall::Wall(const Vec2d& normal, double distanceFromTheOrigin) :
  unitNormal_{unit(normal)},
  ptOnWall_{-distanceFromTheOrigin * unitNormal_} {}

void Wall::resolveCollisions(ParticleSystem& ps) const
{
  for (auto& particle : ps.particles) {
    resolveCollision(particle.pos, particle.vel);
  }
}

void Wall::resolveCollision(Vec2d& pos, Vec2d& vel) const
{
  auto w = pos - ptOnWall_;
  if (dotProduct(w, unitNormal_) > 0) return;
  pos -= 2 * project(w, unitNormal_);
  vel -= 2 * project(vel, unitNormal_);
}

} // end namespace sph