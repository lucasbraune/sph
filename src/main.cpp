#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include <iostream>
#include "util.hpp"
#include "particle_system.hpp"
#include "forces.hpp"
#include "view.hpp"
#include "simulation.hpp"

void display();
void idle();
void keyboard(unsigned char key, int x, int y);

const size_t numberOfParticles = 1000;
const double particleMass = 1.0 / numberOfParticles;
const Rectangle region{-1.0, -1.0, 1.0, 1.0};
PointGravity gravity{Vec2d{0.0, 0.0}, 1.0};
LinearDamping damping{0.001};
const double timeStep = 0.01;
ParticleSystem particleSystem{
    randomVectors(region, numberOfParticles), vector<Vec2d>(numberOfParticles),
    vector<Force*>{&gravity}, &damping, particleMass, timeStep};
Simulation simulation{particleSystem};

const double particleRadius = 0.02;
const View view(region, particleRadius);

int main(int argc, char** argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize(view.windowWidth(), view.windowHeight());
  glutCreateWindow(view.title());

  gluOrtho2D(view.region().xmin, view.region().xmax,
             view.region().ymin, view.region().ymax);

  glutDisplayFunc(display);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);

  simulation.switchPauseState(); // Unpauses the simulation
  glutMainLoop();
  return 0;
}

void display()
{
  view.draw(simulation);  
  glutSwapBuffers();
}

void idle()
{
  simulation.computeNextFrame();
  simulation.waitForNextFrame();
  glutPostRedisplay();
}

void keyboard(unsigned char c, int, int)
{
  switch (c) {
  case 'p':
  case 'P':
    simulation.switchPauseState();
    break;
  case 's':
    simulation.speedDown();
    break;
  case 'S':
    simulation.speedUp();
    break;
  }
}
