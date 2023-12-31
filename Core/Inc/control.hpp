#ifndef CONTROL_H
#define CONTROL_H

#include "math.h"


class Integrator //final
{
public:
  Integrator() {}
  ~Integrator() {}

  void init(float ts);
  void reset();
  void reset(float x);
  void input(float u);
  void step();
  float output();
  float evolve(float u);
  
private:
  float ts;
  float x;
  float u;
};


class Filter //final
{
public:
  void init(float tau, float ts);
  void reset();
  void reset(float x);
  void input(float u);
  void step();
  float output();
  float evolve(float u);

private:
  float tau;

  float u;
  float x;

  float A = 0.0;
  float B = 0.0;
  float C = 0.0;
};


class PID final
{
public:
  void init(float ts, float pole, float sat, bool bumpless);
  void setup(float kp, float ki, float kd);
  void reset();
  void reset(float xi, float xd);
  void input(float e);
  void step();
  float output();
  float evolve(float e);

private:
  void apply_saturation();

  float ts = 0.0;
  float pole = 0.0;
  float sat = 0.0;
  bool bumpless = false;

  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;
  
  float e = 0.0;
  float xi = 0.0;
  float xd = 0.0;
  
  float A = 0.0;
  float B = 0.0;
  float C = 0.0;
  float D = 0.0;
};


#endif  // CONTROL_H
