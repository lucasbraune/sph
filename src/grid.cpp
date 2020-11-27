#include "grid.hpp"
#include <iostream>
#include <cassert>

grid::grid(double xmax, double ymax, double cell_size) :
  m_cell_size(cell_size),
  m_rows(ceil(xmax / cell_size)),
  m_cols(ceil(ymax / cell_size))
{
  m_cell_contents.resize(m_rows);
  for (auto& row : m_cell_contents) {
    row.resize(m_cols);
  }
}

cell_coords grid::get_cell(Vec2d pos) const
{
  size_t r = floor(pos[0] / m_cell_size);
  if (r < 0) r = 0;
  else if (r >= m_rows) r = m_rows-1;
  
  size_t c = floor(pos[1] / m_cell_size);
  if (c < 0) c = 0;
  else if (c >= m_cols) c = m_cols-1;

  return make_pair(r, c);
}

void grid::fill(const vector<Vec2d>& x)
{
  for (auto& row : m_cell_contents) {
    for (auto& cell : row) {
      cell.clear();
    }
  }
  for (size_t i=0; i<x.size(); i++) {
    auto p = get_cell(x[i]);
    m_cell_contents[p.first][p.second].push_back(i);
  }
}

const particle_indices& grid::get_content(cell_coords coords) const
{
  return m_cell_contents[coords.first][coords.second];
}

vector<cell_coords> grid::get_nearby_cells(cell_coords center, double radius) const
{
  vector<cell_coords> nearby_cells {};
  int l = (int) ceil(radius / m_cell_size);
  for (int i=-l; i<=l; i++) {
    int r = center.first + i;
    if (r < 0 || (size_t) r >= m_rows) {
      continue;
    }
    for (int j=-l; j<=l; j++) {
      int c = center.second + j;
      if (c < 0 || (size_t) c >= m_cols) {
        continue;
      }
      nearby_cells.push_back(cell_coords{r, c});
    }
  }
  return nearby_cells;
}

vector<const particle_indices*> grid::get_nearby_particles(Vec2d center, double radius) const 
{
  vector<const particle_indices*> nearby_content;
  vector<cell_coords> nearby_cells = get_nearby_cells(get_cell(center), radius);
  for (auto cell : nearby_cells) {
    nearby_content.push_back(&get_content(cell));
  }
  return nearby_content;
}

grid_iterator::grid_iterator(
    vector<const particle_indices*> nearby_particles) :
  m_nearby_particles(nearby_particles),
  m_i(0),
  m_j(0)
{
  assert(m_i < m_nearby_particles.size());
  while ((*nearby_particles[m_i]).size() == 0) {
    ++m_i;
    assert(m_i < m_nearby_particles.size());
  }
}

bool grid_iterator::has_next() const
{
  return m_i < m_nearby_particles.size()
      && m_j < m_nearby_particles[m_i]->size();
}

size_t grid_iterator::next()
{
  size_t res = (*m_nearby_particles[m_i])[m_j];
  m_j++;
  if (m_j >= m_nearby_particles[m_i]->size()) {
    m_j = 0;
    m_i++;
  }
  return res;
}

grid_loop_util::grid_loop_util(
    double interaction_radius, unique_ptr<grid> g) :
  m_interaction_radius(interaction_radius),
  m_grid(*g) 
{}

void grid_loop_util::refresh(const vector<Vec2d>& x)
{
  m_grid.fill(x);
}

unique_ptr<neighbor_iterator> grid_loop_util::build(Vec2d position) const
{
  vector<const particle_indices*> nearby_particles =
      m_grid.get_nearby_particles(position, m_interaction_radius);
  return std::make_unique<grid_iterator>(nearby_particles);
}

unique_ptr<neighbor_loop_util> grid_loop_util::clone() const
{
  return std::make_unique<grid_loop_util>(*this);
}