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

void drawText(std::string line)
{
  for (size_t i=0; i<line.size(); i++) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, line[i]);
  }
}

void View::draw(const ParticleSystem& ps) const
{
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);

  glColor3f(0.0, 0.0, 0.0);
  for (auto position : ps.positions()) {
    glBegin(GL_POLYGON);
    for (auto vertex : particlePolygon_) {
      Vec2d translate = vertex + position;
      glVertex2d(translate[0], translate[1]);
    }
    glEnd();
  }

  glColor3f(1.0, 0.0, 0.0);
  glRasterPos2d(0.95 * region().xmin, 0.95 * region().ymin);
  drawText("t = " + std::to_string(ps.time()));
}