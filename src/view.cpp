#include "view.hpp"
#include <cmath>

RegularPolygon::RegularPolygon(Vec2d center, double radius, size_t sides)
{
  for (size_t i=0; i<sides; i++) {
    double theta = 2 * M_PI * i / sides;
    vertices_.push_back(center + radius * Vec2d{cos(theta), sin(theta)});
  }
}

RegularPolygon::RegularPolygon(double radius, size_t sides) :
  RegularPolygon(Vec2d{0.0, 0.0}, radius, sides)
{}

const vector<Vec2d>& RegularPolygon::vertices() const
{
  return vertices_;  
}


View::View(const Rectangle region, const int windowHeight, const string title,
           const vector<Vec2d> particlePolygon) :
  region_(region),
  particlePolygon_(particlePolygon),
  heightInPixels_(windowHeight),
  title_(title)
{}

View::View(const Rectangle region, const int windowHeight, const string title,
           const double radius, const size_t sides) :
  View(region, windowHeight, title, RegularPolygon(radius, sides).vertices())
{}

void drawText(std::string line)
{
  for (size_t i=0; i<line.size(); i++) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, line[i]);
  }
}

string textInfo(const Simulation& sim)
{
  string text =
      "time = " + std::to_string(sim.time()) + 
      ", speed = " + std::to_string(sim.speed()) + "x";
  if (sim.paused()) {
    text.append(", PAUSED");
  } 
  return text;
}

void View::draw(const Simulation& sim) const
{
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);

  glColor3f(0.0, 0.0, 0.0);
  for (auto position : sim.positions()) {
    glBegin(GL_POLYGON);
    for (auto vertex : particlePolygon_) {
      Vec2d translate = vertex + position;
      glVertex2d(translate[0], translate[1]);
    }
    glEnd();
  }

  glColor3f(1.0, 0.0, 0.0);
  glRasterPos2d(0.95 * region().xmin, 0.95 * region().ymin);
  drawText(textInfo(sim));
}

Rectangle View::region() const
{
  return region_;
}

int View::windowHeight() const 
{
  return heightInPixels_;
}

int View::windowWidth() const
{
  double width = region_.xmax - region_.xmin;
  double height = region_.ymax - region_.ymin;
  return (int) ((width / height) * heightInPixels_);
}

const char* View::title() const
{
  return title_.c_str();
}