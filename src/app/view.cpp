#define GL_SILENCE_DEPRECATION

#include "view.hpp"
#include <cmath> // M_PI, cos, sin
#include <GL/freeglut.h>

using std::vector;
using std::string;
using namespace sph;

RegularPolygon::RegularPolygon(Vec2d center, double radius, size_t sides)
{
  for (size_t i=0; i<sides; i++) {
    double theta = 2 * M_PI * i / sides;
    vertices_.push_back(center + radius * Vec2d{std::cos(theta), std::sin(theta)});
  }
}

RegularPolygon::RegularPolygon(double radius, size_t sides) :
  RegularPolygon(Vec2d{0.0, 0.0}, radius, sides)
{}

const vector<Vec2d>& RegularPolygon::vertices() const
{
  return vertices_;  
}

View::View(const Rectangle region, const int windowHeight, 
           const string title, const vector<Vec2d> particlePolygon) :
  region_(region),
  particlePolygon_(particlePolygon),
  heightInPixels_(windowHeight),
  title_(title)
{}

View::View(const Rectangle region, const double particleRadius, const size_t sides,
           const int windowHeight, const string title) :
  View(region, windowHeight, title, RegularPolygon(particleRadius, sides).vertices())
{}

namespace {

void drawText(std::string line)
{
for (size_t i=0; i<line.size(); i++) {
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, line[i]);
}
}

string textInfo(const SimulationState& state)
{
  string text =
      "time = " + std::to_string(state.ps.time) + 
      ", target speed = " + std::to_string(state.targetSpeed) + "x";
  if (state.paused) {
    text.append(", PAUSED");
  } 
  return text;
}

} // namespace

void View::draw(const SimulationState& state) const
{
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);

  glColor3f(0.0, 0.0, 0.0);
  for (auto& particle : state.ps.particles) {
    glBegin(GL_POLYGON);
    for (auto vertex : particlePolygon_) {
      Vec2d translate = vertex + particle.pos;
      glVertex2d(translate[0], translate[1]);
    }
    glEnd();
  }

  glColor3f(1.0, 0.0, 0.0);
  glRasterPos2d(0.95 * region_.xmin, 0.95 * region_.ymin);
  drawText(textInfo(state));
}

int View::windowWidth() const
{
  return static_cast<int>((width(region_) / height(region_)) * heightInPixels_);
}