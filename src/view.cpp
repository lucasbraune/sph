#include "view.hpp"
#include <cmath>

View::View(const Rectangle region, const vector<Vec2d> particlePolygon) :
  region_(region),
  particlePolygon_(particlePolygon)
{}

View::View(const Rectangle region, const double radius, const size_t sides) :
  View(region, RegularPolygon(radius, sides).vertices())
{}

Rectangle View::region() const
{
  return region_;
}

void View::draw(const vector<Vec2d>& positions) const
{
  for (auto position : positions) {
    glBegin(GL_POLYGON);
    for (auto vertex : particlePolygon_) {
      Vec2d translate = vertex + position;
      glVertex2d(translate[0], translate[1]);
    }
    glEnd();
  }
}