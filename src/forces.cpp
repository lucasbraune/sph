#include "forces.hpp"

PointGravity::PointGravity(const Vec2d center, const double gravityConstant) :
  center_(center), intensity_(gravityConstant)
{}

void PointGravity::apply(const double, const double, const vector<Vec2d>& positions,
                         vector<Vec2d>& accelerations) const
{
  for (size_t i=0; i<positions.size(); i++) {
    accelerations[i] -= intensity_ * (positions[i] - center_);
  }
}

void PointGravity::setConstant(double newValue)
{
  intensity_ = newValue;
}

void PointGravity::increase()
{
  intensity_ *= 2.0;
}

void PointGravity::decrease()
{
  intensity_ *= 0.5;
}

LinearDamping::LinearDamping(const double dampingConstant) :
  intensity_(dampingConstant)
{}

Vec2d LinearDamping::acceleration(const double, const double, const Vec2d velocity) const
{
  return -intensity_ * velocity;
}

void LinearDamping::increase()
{
  intensity_ *= 2.0;
}

void LinearDamping::decrease()
{
  intensity_ *= 0.5;
}