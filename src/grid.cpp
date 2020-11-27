// #include "grid.hpp"
// #include <iostream>
// #include <cassert>

// Grid::Grid(Rectangle region, double cellSize) :
//   region_(region),
//   cellSize_(cellSize),
//   rows_(ceil(region_.width() / cellSize_)),
//   cols_(ceil(region_.height() / cellSize_)),
//   matrix_(rows_, vector<Indices>(cols_))
// {}

// CellCoords Grid::cell(Vec2d pos) const
// {
//   Vec2d diff = pos - region_.bottomLeft();
  
//   int r = floor(pos[0] / cellSize_);
//   if (r < 0) r = 0;
//   else if (r >= rows_) r = rows_-1;
  
//   int c = floor(pos[1] / cellSize_);
//   if (c < 0) c = 0;
//   else if (c >= cols_) c = cols_-1;

//   return CellCoords(r, c);
// }

// void Grid::update(const vector<Vec2d>& positions)
// {
//   for (auto& row : matrix_) {
//     for (auto& cell : row) {
//       cell.clear();
//     }
//   }
//   for (size_t i=0; i<positions.size(); i++) {
//     auto p = cell(positions[i]);
//     matrix_[p.first][p.second].push_back(i);
//   }
// }

// const Indices& Grid::content(CellCoords coords) const
// {
//   return matrix_[coords.first][coords.second];
// }

// vector<CellCoords> Grid::nearbyCells(CellCoords cell, double radius) const
// {
//   vector<CellCoords> nearbyCells {};
//   const int L = (int) ceil(radius / cellSize_);
//   for (int i=-L; i<=L; i++) {
//     int r = cell.first + i;
//     if (r < 0 || (size_t) r >= rows_) {
//       continue;
//     }
//     for (int j=-L; j<=L; j++) {
//       int c = cell.second + j;
//       if (c < 0 || (size_t) c >= cols_) {
//         continue;
//       }
//       nearbyCells.push_back(CellCoords{r, c});
//     }
//   }
//   return nearbyCells;
// }

// Indices Grid::nearbyParticles(Vec2d center, double radius) const 
// {
//   Indices result;
//   for (auto cell : nearbyCells(cell(center), radius)) {
//     auto& c = content(cell);
//     result.insert(result.end(), c.begin(), c.end());
//   }
//   return result;
// }

// GridIterator::GridIterator(
//     vector<const Indices*> nearby_particles) :
//   nearbyParticles_(nearby_particles),
//   i_(0),
//   j_(0)
// {
//   assert(i_ < nearbyParticles_.size());
//   while ((*nearby_particles[i_]).size() == 0) {
//     ++i_;
//     assert(i_ < nearbyParticles_.size());
//   }
// }

// bool GridIterator::hasNext() const
// {
//   return i_ < nearbyParticles_.size()
//       && j_ < nearbyParticles_[i_]->size();
// }

// size_t GridIterator::next()
// {
//   size_t res = (*nearbyParticles_[i_])[j_];
//   j_++;
//   if (j_ >= nearbyParticles_[i_]->size()) {
//     j_ = 0;
//     i_++;
//   }
//   return res;
// }

// GridNeighborIteratorFactory::GridNeighborIteratorFactory(
//     double interaction_radius, unique_ptr<Grid> g) :
//   interactionRadius_(interaction_radius),
//   grid_(*g) 
// {}

// void GridNeighborIteratorFactory::refresh(const vector<Vec2d>& positions)
// {
//   grid_.update(positions);
// }

// unique_ptr<Iterator> GridNeighborIteratorFactory::build(Vec2d position) const
// {
//   vector<const Indices*> nearby_particles =
//       grid_.nearbyParticleIndices(position, interactionRadius_);
//   return std::make_unique<GridIterator>(nearby_particles);
// }

// unique_ptr<NeighborIteratorFactory> GridNeighborIteratorFactory::clone() const
// {
//   return std::make_unique<GridNeighborIteratorFactory>(*this);
// }