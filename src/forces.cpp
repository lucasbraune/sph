#include "forces.hpp"

PointGravity::PointGravity(const Vec2d center, const double nu) :
  center_(center), intensity_(nu)
{}

void PointGravity::apply(const double, const double, const vector<Vec2d>& positions,
                         vector<Vec2d>& accelerations) const
{
  for (size_t i=0; i<positions.size(); i++) {
    accelerations[i] -= intensity_ * (positions[i] - center_);
  }
}

LinearDamping::LinearDamping(const double intensity) :
  intensity_(intensity)
{}

Vec2d LinearDamping::acceleration(const double, const double, const Vec2d velocity) const
{
  return -intensity_ * velocity;
}