#include "view.hpp"

View::View(const Rectangle region, const double particleRadius) : 
    region_(region),
    particleRadius_(particleRadius)
{}

Rectangle View::region() const
{
  return region_;
}

void View::draw(const vector<Vec2d>& positions) const
{
  for (auto position : positions) {
    drawDisk(position, particleRadius_);
  }
}

void View::drawDisk(const Vec2d center, const double radius) const
{
  // Approximates a circle by an octagon
  constexpr double slope = 0.70710678118; // cos(pi/4)
  glBegin(GL_POLYGON);
  glVertex2d(center[0] + radius         , center[1]                 );
  glVertex2d(center[0] + slope * radius , center[1] + slope * radius);
  glVertex2d(center[0]                  , center[1] + radius        );
  glVertex2d(center[0] - slope * radius , center[1] + slope * radius);
  glVertex2d(center[0] - radius         , center[1]                 );
  glVertex2d(center[0] - slope * radius , center[1] - slope * radius);
  glVertex2d(center[0]                  , center[1] - radius        );
  glVertex2d(center[0] + slope * radius , center[1] - slope * radius);
  glEnd();
}


