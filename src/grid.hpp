// #ifndef GRID_H
// #define GRID_H

// #include <vector>
// #include <cmath>
// #include "pressure_force.hpp"

// using std::vector;

// template <typename E>
// using Matrix = vector<vector<E>>;
// using CellCoords = std::pair<size_t, size_t>;
// using Indices = vector<size_t>;

// class Grid {
// public:
//   Grid(Rectangle region, double cell_size);
//   void update(const vector<Vec2d>& positions);
//   /** Returns a list of particle indices that includes all particles within a given disk. */
//   Indices nearbyParticles(Vec2d center, double radius) const;

// private:
//   CellCoords cell(Vec2d position) const;
//   const Indices& content(CellCoords cell) const;
//   vector<CellCoords> nearbyCells(CellCoords cell, double radius) const;

//   Rectangle region_;
//   const double cellSize_;
//   const size_t rows_, cols_;
//   Matrix<Indices> matrix_;
// };

// class GridIterator : public NeighborIterator {
// public:
//   GridIterator(const Indices& nearbyParticles); 
//   bool hasNext() const override;
//   size_t next() override;

// private:
//   Indices nearbyParticles_;
//   size_t i_, j_;
// };

// class GridNeighborIteratorFactory : public NeighborIteratorFactory {
// public:
//   GridNeighborIteratorFactory(double interactionRadius, unique_ptr<Grid> grid);
//   void refresh(const vector<Vec2d>& x) override;
//   unique_ptr<Iterator> build(Vec2d position) const override;
//   unique_ptr<NeighborIteratorFactory> clone() const override;

// private:
//   const double interactionRadius_;
//   Grid grid_;
// };



// #endif