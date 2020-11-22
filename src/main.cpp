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

constexpr int WINDOW_SIZE = 750;
constexpr char WINDOW_TITLE[] = "Fluid simulation";

const int fps = 60;
const double playbackSpeed = 1.0;
TimeController timeController{fps, playbackSpeed};

const size_t numberOfParticles = 1000;
const double particleMass = 1.0 / numberOfParticles;
const Rectangle region{-1.0, -1.0, 1.0, 1.0};
PointGravity gravity{Vec2d{0.0, 0.0}, 1.0};
LinearDamping damping{0.1};
const double timeStep = 0.01;

const vector<Force*> forces{&gravity};
const vector<Damping*> dampings{&damping};
ParticleSystem particleSystem{
    randomPositions(region, numberOfParticles), vector<Vec2d>(numberOfParticles),
    forces, dampings, particleMass, timeStep};

const double particleRadius = 0.02;
const int sides = 12;
const View view(region, particleRadius, sides);

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

  timeController.setStartOfSimulation();
  glutMainLoop();
  return 0;
}

void display()
{
  view.draw(particleSystem);  
  glutSwapBuffers();
}

void idle()
{
  particleSystem.integrate(timeController.timeUntilNextFrame());
  timeController.waitUntil(particleSystem.time());
  glutPostRedisplay();
}

void keyboard(unsigned char, int, int) {}
