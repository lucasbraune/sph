# sph-fluids
This program simulates a two-dimensional fluid using the SPH method. SPH stands for <a href="https://en.wikipedia.org/wiki/Smoothed-particle_hydrodynamics">smoothed particle hydrodynamics</a>. The simulation runs in real time. Fluid particles are drawn on the screen using OpenGL. 

# Demo
<img src="toystar.gif" width="600">

# Building and running the program

Requires C++17, GNU Make and <a href="http://freeglut.sourceforge.net">freeglut</a>.
On MacOS, using <a href="https://brew.sh">Homebrew</a>, freeglut can be installed with
```
brew install freeglut
```
To build and run the program, execute the command
```
make run
```
