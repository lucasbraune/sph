#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include <iostream>
#include "util.hpp"
#include "particle_system.hpp"
#include "view.hpp"

void display();
void idle();
void keyboard(unsigned char key, int x, int y);

constexpr int WINDOW_SIZE = 600;
constexpr char WINDOW_TITLE[] = "Fluid simulation";

const size_t N = 1000;
const int targetFps = 60;
const Rectangle region{-1.0, -1.0, 1.0, 1.0};
const double particleRadius = 0.02;

constexpr Vec2d ZERO_VECTOR{};
const double nu = 0.001;
PointGravity gravity{ZERO_VECTOR, nu};
vector<Force*> forces{&gravity};

const View view(region, particleRadius);
ParticleSystem particleSystem{randomPositions(region, N), vector<Vec2d>(N), forces, 1.0 / N,
    1.0 / targetFps};

int main(int argc, char** argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize(WINDOW_SIZE, WINDOW_SIZE);
  glutCreateWindow(WINDOW_TITLE);

  gluOrtho2D(view.region().xmin, view.region().xmax,
             view.region().ymin, view.region().ymax);

  glutDisplayFunc(display);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);
  glutMainLoop();
  return 0;
}

void display()
{
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);

  glColor3f(0.0, 0.0, 0.0);
  view.draw(particleSystem.positions());

  glutSwapBuffers();
}

void idle()
{
  particleSystem.integrate(1.0 / targetFps);
  std::cout << particleSystem.time() << "\n";

  glutPostRedisplay();
}

void keyboard(unsigned char, int, int) {}
