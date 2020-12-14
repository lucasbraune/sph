#define GL_SILENCE_DEPRECATION

#include <GL/freeglut.h>
#include "demo.hpp"
#include "view.hpp"
#include "controller.hpp"

void display();
void idle();
void keyboard(unsigned char key, int x, int y);

ToyStarSimulation simulation;
const View view(simulation);
const Controller controller(simulation);

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

  simulation.togglePause(); // Unpauses the simulation
  glutMainLoop();
  return 0;
}

void display()
{
  view.draw();  
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
  controller.handleKeyboardInput(c);
}
