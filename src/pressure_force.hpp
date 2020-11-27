#ifndef PRESSURE_FORCE_H
#define PRESSURE_FORCE_H

#include <vector>
#include <memory>
#include <functional>
#include <cmath>
#include "util.hpp"

using std::vector;
using std::unique_ptr;
using std::function;

class NeighborIterator {
  public:
  virtual bool hasNext() const = 0;
  virtual size_t next() = 0;
  virtual ~NeighborIterator() {};
};

class NeighborIteratorFactory {
  public:
  virtual void refresh(const vector<Vec2d>& positions) = 0; // when particles move
  virtual unique_ptr<NeighborIterator> build(Vec2d position) const = 0;

  virtual unique_ptr<NeighborIteratorFactory> clone() const = 0;
  virtual ~NeighborIteratorFactory() {};
};

class SmoothingKernel {
  public:
  virtual double operator()(double dist) const = 0;
  virtual double DifferentiatedAt(double dist) const = 0;
  virtual double interactionRadius() const = 0;
  
  virtual unique_ptr<SmoothingKernel> clone() const = 0;
  virtual ~SmoothingKernel() {};
};

class PressureForce {
  public:
  PressureForce(unique_ptr<NeighborIteratorFactory> iteratorFactory,
                unique_ptr<SmoothingKernel> kernel,
                function<double(double)> pressure);
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

class CubicKernel : public SmoothingKernel {
  public:
  CubicKernel(double smoothingLength);
  double operator()(double dist) const override;
  double DifferentiatedAt(double dist) const override;
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