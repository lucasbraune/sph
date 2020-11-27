#ifndef GRID_H
#define GRID_H

#include <vector>
#include <cmath>
#include "pressure_force.hpp"

using std::vector;
using std::pair;
using std::make_pair;

using cell_coords = pair<size_t, size_t>;
using particle_indices = vector<size_t>;

class grid {
public:
  grid(double xmax, double ymax, double cell_size);
  void fill(const vector<Vec2d>& positions);
  cell_coords get_cell(Vec2d position) const;
  const particle_indices& get_content(cell_coords cell) const;
  vector<cell_coords> get_nearby_cells(cell_coords cell, double radius) const;
  vector<const particle_indices*> get_nearby_particles(Vec2d center, double radius) const;

private:
  vector<vector<particle_indices>> m_cell_contents;
  const double m_cell_size;
  const size_t m_rows, m_cols;
};

class grid_iterator : public neighbor_iterator {
public:
  grid_iterator(vector<const particle_indices*> nearby_particles); 
  bool has_next() const override;
  size_t next() override;

private:
  vector<const particle_indices*> m_nearby_particles;
  size_t m_i, m_j;
};

class grid_loop_util : public neighbor_loop_util {
public:
  grid_loop_util(double interaction_radius, unique_ptr<grid> g);
  void refresh(const vector<Vec2d>& x) override;
  unique_ptr<neighbor_iterator> build(Vec2d position) const override;
  unique_ptr<neighbor_loop_util> clone() const override;

private:
  const double m_interaction_radius;
  grid m_grid;
};



#endif