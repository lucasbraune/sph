#ifndef VIEW_HPP
#define VIEW_HPP

#include "simulation.hpp"
#include "sample_simulations.hpp"

namespace sph {

class View {
public:
  View(const Rectangle& region, size_t numberOfParticles,
       double density = 0.3, size_t sides = 10);
  void draw(const SimulationState& state) const;
  const Rectangle region;

private:
  const std::vector<Vec2d> particlePolygon_;
};

namespace detail {

template<class Simulation>
void keyboardHelper(Simulation& simulation, char c)
{
  switch (c) {
  case 'p':
  case 'P':
    simulation.togglePause();
    break;
  case 's':
    simulation.decreaseTargetSpeed();
    break;
  case 'S':
    simulation.increaseTargetSpeed();
    break;
  }
}

} // namespace detail

template<class Simulation>
void handleKeyboardInput(Simulation& simulation, char c)
{
  detail::keyboardHelper(simulation, c);
}

template<>
inline void handleKeyboardInput(BreakingDamSimulation& simulation, char c)
{
  switch (c) {
  case 'b':
  case 'B':
    simulation.breakDam();
    break;
  case 'd':
    simulation.decreaseDamping();
    break;
  case 'D':
    simulation.increaseDamping();
    break;
  default:
    detail::keyboardHelper(simulation, c);
  }
}

} // namespace sph

#endif