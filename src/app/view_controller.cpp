#define GL_SILENCE_DEPRECATION

#include <cmath> // M_PI, cos, sin, sqrt
#include <string>
#include <GL/freeglut.h>
#include "view_controller.hpp"

using namespace sph;

namespace {

std::vector<Vec2d> regularPolygon(double radius, size_t sides)
{
  auto vertices = std::vector<Vec2d>{};
  for (size_t i=0; i<sides; ++i) {
    auto theta = 2 * M_PI * i / sides;
    vertices.push_back(radius * Vec2d{std::cos(theta), std::sin(theta)});
  }
  return vertices;
}

double particleRadius(const Rectangle& region, size_t numberOfParticles, double density)
{
  const auto area = height(region) * width(region);
  return std::sqrt(density * area / (2 * M_PI * numberOfParticles));
}

void drawText(const std::string& line)
{
  for (size_t i=0; i<line.size(); i++) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, line.at(i));
  }
}

std::string textInfo(const SimulationState& state)
{
  auto text = "time = " + std::to_string(state.ps.time) + 
              ", target speed = " + std::to_string(state.targetSpeed) + "x";
  if (state.paused) text.append(", PAUSED");
  return text;
}

} // namespace

DrawFunction::DrawFunction(const Rectangle& region, size_t numberOfParticles, double density, size_t sides) :
  region_{region},
  particlePolygon_{regularPolygon(particleRadius(region, numberOfParticles, density), sides)}
{}

void DrawFunction::operator()(const SimulationState& state) const
{
  glClearColor(1.0, 1.0, 1.0, 1.0); // White
  glClear(GL_COLOR_BUFFER_BIT);

  glColor3f(0.0, 0.0, 0.0); // Black
  for (const auto& particle : state.ps.particles) {
    glBegin(GL_POLYGON);
    for (auto& vertex : particlePolygon_) {
      Vec2d translate = vertex + particle.pos;
      glVertex2d(translate[0], translate[1]);
    }
    glEnd();
  }

  glColor3f(1.0, 0.0, 0.0); // Red
  glRasterPos2d(0.95 * region_.xmin, 0.95 * region_.ymin);
  drawText(textInfo(state));
}