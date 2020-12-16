#include "physics_elements.hpp"

using std::vector;

namespace sph {

PointGravity::PointGravity(double gravityConstant, const Vec2d& center) :
  center_(center), intensity_(gravityConstant)
{}

void PointGravity::apply(const vector<Vec2d>& positions, double, double,
                         vector<Vec2d>& accelerations) const
{
  for (size_t i=0; i<positions.size(); i++) {
    accelerations[i] -= intensity_ * (positions[i] - center_);
  }
}

SurfaceGravity::SurfaceGravity(double magnitude) :
  acceleration_{0.0, -magnitude} {}

void SurfaceGravity::apply(const vector<Vec2d>&, double, double, vector<Vec2d>& accelerations) const
{
  for (auto& acc : accelerations) {
    acc += acceleration_;
  }
}

LinearDamping::LinearDamping(double dampingConstant) :
  intensity_(dampingConstant)
{}

Vec2d LinearDamping::acceleration(const Vec2d& velocity, double mass) const
{
  return -intensity_ * velocity / mass;
}

Wall::Wall(const Vec2d& normal, double distanceFromTheOrigin) :
  unitNormal_{unit(normal)},
  ptOnWall_{-distanceFromTheOrigin * unitNormal_} {}

void Wall::resolveCollisions(vector<Vec2d>& positions, vector<Vec2d>& velocities, double) const
{
  for (size_t i=0; i<positions.size(); i++) {
    resolveCollision(positions[i], velocities[i]);
  }
}

void Wall::resolveCollision(Vec2d& pos, Vec2d& vel) const
{
  auto w = pos - ptOnWall_;
  if (w * unitNormal_ > 0) return;
  pos -= 2 * project(w, unitNormal_);
  vel -= 2 * project(vel, unitNormal_);
}

} // end namespace sph