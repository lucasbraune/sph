#ifndef GRID_H
#define GRID_H

#include <vector>
#include <cmath>
#include "pressure_force.hpp"

using std::vector;
using std::pair;
using std::make_pair;

using CellCoords = pair<size_t, size_t>;
using ParticleIndices = vector<size_t>;

class Grid {
public:
  Grid(double xmax, double ymax, double cell_size);
  void fill(const vector<Vec2d>& positions);
  CellCoords cell(Vec2d position) const;
  const ParticleIndices& content(CellCoords cell) const;
  vector<CellCoords> nearbyCells(CellCoords cell, double radius) const;
  vector<const ParticleIndices*> nearbyParticleIndices(Vec2d center, double radius) const;

private:
  vector<vector<ParticleIndices>> cellContents_;
  const double cellSize_;
  const size_t rows_, cols_;
};

class GridIterator : public NeighborIterator {
public:
  GridIterator(vector<const ParticleIndices*> nearbyParticles); 
  bool hasNext() const override;
  size_t next() override;

private:
  vector<const ParticleIndices*> nearbyParticles_;
  size_t i_, j_;
};

class GridNeighborIteratorFactory : public NeighborIteratorFactory {
public:
  GridNeighborIteratorFactory(double interactionRadius, unique_ptr<Grid> grid);
  void refresh(const vector<Vec2d>& x) override;
  unique_ptr<NeighborIterator> build(Vec2d position) const override;
  unique_ptr<NeighborIteratorFactory> clone() const override;

private:
  const double interactionRadius_;
  Grid grid_;
};



#endif