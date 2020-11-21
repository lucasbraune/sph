#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include <iostream>
#include "util.hpp"
#include "particle_system.hpp"
#include "view.hpp"
#include "time_controller.hpp"

void display();
void idle();
void keyboard(unsigned char key, int x, int y);

constexpr int WINDOW_SIZE = 600;
constexpr char WINDOW_TITLE[] = "Fluid simulation";

TimeController timeController{60};

const size_t numberOfParticles = 1000;
const Rectangle region{-1.0, -1.0, 1.0, 1.0};

PointGravity gravity{Vec2d{0.0, 0.0}, 1.0};
vector<Force*> forces{&gravity};

const View view(region, 0.02, 12);
ParticleSystem particleSystem{
    randomPositions(region, numberOfParticles),
    vector<Vec2d>(numberOfParticles),
    forces,
    1.0 / numberOfParticles,
    timeController.timeUntilNextFrame()};

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

  timeController.setStart();
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
  particleSystem.integrate(timeController.timeUntilNextFrame());
  timeController.waitUntil(particleSystem.time());
  std::cout << particleSystem.time() << "\n";

  glutPostRedisplay();
}

void keyboard(unsigned char, int, int) {}
