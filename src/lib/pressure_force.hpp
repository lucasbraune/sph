#ifndef PRESSURE_FORCE_H
#define PRESSURE_FORCE_H

#include <vector>
#include <memory>
#include <functional>
#include <cmath>
#include "util.hpp"
#include "particle_system.hpp"

using std::vector;
using std::unique_ptr;
using std::function;

template<typename T>
class Iterator {
public:
  virtual bool hasNext() const = 0;
  virtual T next() = 0;
  virtual ~Iterator() {};
};

using NeighborIterator = Iterator<size_t>;

class NeighborIteratorFactory {
public:
  virtual void refresh(const vector<Vec2d>& positions) = 0; // when particles move
  virtual unique_ptr<NeighborIterator> build(Vec2d position) const = 0;
  virtual unique_ptr<NeighborIteratorFactory> clone() const = 0;
  virtual ~NeighborIteratorFactory() {};
};

class SmoothingKernel {
  public:
  virtual double operator()(Vec2d x) const = 0;
  virtual Vec2d gradientAt(Vec2d x) const = 0;
  virtual double interactionRadius() const = 0;
  
  virtual unique_ptr<SmoothingKernel> clone() const = 0;
  virtual ~SmoothingKernel() {};
};

class PressureForce : public Force {
public:
  PressureForce(unique_ptr<NeighborIteratorFactory> iteratorFactory,
                unique_ptr<SmoothingKernel> kernel,
                function<double(double)> pressure);
  PressureForce(double interactionRadius, function<double(double)> pressure);
  PressureForce(const PressureForce& other); 
  PressureForce(PressureForce&& other) = default;
  PressureForce& operator=(const PressureForce& other);
  PressureForce& operator=(PressureForce&& other) = default;
  ~PressureForce() = default;
  
  void apply(const double time, const double particleMass, const vector<Vec2d>& positions,
             vector<Vec2d>& accelerations) const;

private:
  vector<double> computeDensities(double particleMass, const vector<Vec2d>& positions) const;

  unique_ptr<SmoothingKernel> kernel_;
  unique_ptr<NeighborIteratorFactory> neighborIteratorFactory_;
  function<double(double)> pressure_;
};

template<typename T>
class RangeIterator : public Iterator<T> {
public:
  RangeIterator(T begin, T end);
  bool hasNext() const override;
  T next() override;

private:
  const T begin_, end_;
  T next_;
};

class TrivialNeighborIteratorFactory : public NeighborIteratorFactory {
public:
  TrivialNeighborIteratorFactory(size_t numberOfParticles = 0);
  void refresh(const vector<Vec2d>& positions);
  unique_ptr<NeighborIterator> build(Vec2d position) const;
  unique_ptr<NeighborIteratorFactory> clone() const;

private:
  size_t numberOfParticles_;
};

class CubicKernel : public SmoothingKernel {
public:
  CubicKernel(double smoothingLength);
  double operator()(Vec2d x) const override;
  Vec2d gradientAt(Vec2d x) const override;
  double interactionRadius() const override;

  unique_ptr<SmoothingKernel> clone() const override;

private:
  const double smoothingLength_;
  const double C_, D_;
};

class WaterPressure {
public:
  WaterPressure(double pressureConstant, double restDensity);
  double operator()(double density) const;

private:
  const double pressureConstant_;
  const double restDensity_;
};

class GasPressure {
public:
  GasPressure(double pressureConstant);
  double operator()(double density) const;

private:
  const double pressureConstant_;
};

#endif