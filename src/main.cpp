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
const Rectangle region{-1.0, -1.0, 1.0, 1.0};
const double particleRadius = 0.02;

const View view(region, particleRadius);
ParticleSystem particles{randomPositions(region, N)};

int main(int argc, char** argv) {
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

void display() {
  view.draw(particles.positions());
  glutSwapBuffers();
}

void idle() {}

void keyboard(unsigned char, int, int) {}
