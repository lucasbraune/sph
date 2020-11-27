#include "grid.hpp"
#include <iostream>
#include <cassert>

Grid::Grid(double xmax, double ymax, double cellSize) :
  cellSize_(cellSize),
  rows_(ceil(xmax / cellSize)),
  cols_(ceil(ymax / cellSize))
{
  cellContents_.resize(rows_);
  for (auto& row : cellContents_) {
    row.resize(cols_);
  }
}

CellCoords Grid::cell(Vec2d pos) const
{
  size_t r = floor(pos[0] / cellSize_);
  if (r < 0) r = 0;
  else if (r >= rows_) r = rows_-1;
  
  size_t c = floor(pos[1] / cellSize_);
  if (c < 0) c = 0;
  else if (c >= cols_) c = cols_-1;

  return make_pair(r, c);
}

void Grid::fill(const vector<Vec2d>& positions)
{
  for (auto& row : cellContents_) {
    for (auto& cell : row) {
      cell.clear();
    }
  }
  for (size_t i=0; i<positions.size(); i++) {
    auto p = cell(positions[i]);
    cellContents_[p.first][p.second].push_back(i);
  }
}

const ParticleIndices& Grid::content(CellCoords coords) const
{
  return cellContents_[coords.first][coords.second];
}

vector<CellCoords> Grid::nearbyCells(CellCoords cell, double radius) const
{
  vector<CellCoords> nearbyCells {};
  int l = (int) ceil(radius / cellSize_);
  for (int i=-l; i<=l; i++) {
    int r = cell.first + i;
    if (r < 0 || (size_t) r >= rows_) {
      continue;
    }
    for (int j=-l; j<=l; j++) {
      int c = cell.second + j;
      if (c < 0 || (size_t) c >= cols_) {
        continue;
      }
      nearbyCells.push_back(CellCoords{r, c});
    }
  }
  return nearbyCells;
}

vector<const ParticleIndices*> Grid::nearbyParticleIndices(Vec2d center, double radius) const 
{
  vector<const ParticleIndices*> nearbyContent;
  vector<CellCoords> cells = nearbyCells(cell(center), radius);
  for (auto cell : cells) {
    nearbyContent.push_back(&content(cell));
  }
  return nearbyContent;
}

GridIterator::GridIterator(
    vector<const ParticleIndices*> nearby_particles) :
  nearbyParticles_(nearby_particles),
  i_(0),
  j_(0)
{
  assert(i_ < nearbyParticles_.size());
  while ((*nearby_particles[i_]).size() == 0) {
    ++i_;
    assert(i_ < nearbyParticles_.size());
  }
}

bool GridIterator::hasNext() const
{
  return i_ < nearbyParticles_.size()
      && j_ < nearbyParticles_[i_]->size();
}

size_t GridIterator::next()
{
  size_t res = (*nearbyParticles_[i_])[j_];
  j_++;
  if (j_ >= nearbyParticles_[i_]->size()) {
    j_ = 0;
    i_++;
  }
  return res;
}

GridNeighborIteratorFactory::GridNeighborIteratorFactory(
    double interaction_radius, unique_ptr<Grid> g) :
  interactionRadius_(interaction_radius),
  grid_(*g) 
{}

void GridNeighborIteratorFactory::refresh(const vector<Vec2d>& positions)
{
  grid_.fill(positions);
}

unique_ptr<NeighborIterator> GridNeighborIteratorFactory::build(Vec2d position) const
{
  vector<const ParticleIndices*> nearby_particles =
      grid_.nearbyParticleIndices(position, interactionRadius_);
  return std::make_unique<GridIterator>(nearby_particles);
}

unique_ptr<NeighborIteratorFactory> GridNeighborIteratorFactory::clone() const
{
  return std::make_unique<GridNeighborIteratorFactory>(*this);
}