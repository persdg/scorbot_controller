#ifndef CONTROL_H
#define CONTROL_H

#include "math.h"
#include "racs_services/msg/debug.h"


class Integrator //final
{
public:
  Integrator() {}
  ~Integrator() {}

  void init(float ts, float sat);
  void reset();
  void reset(float x);
  void input(float u);
  void step();
  float output();
  float evolve(float u);
  void apply_saturation();
  
private:
  float ts;
  float sat;
  float x;
  float u;
};


class Filter //final
{
public:
  Filter(){}
  void init(float b1, float b0, float a1, float a0, float ts);
  void reset();
  void reset(float x);
  void input(float u);
  void step();
  float output();
  float evolve(float u);

private:

  float a1 = 0;
  float a0 = 0;
  float b1 = 0;
  float b0 = 0;

  float A = 0.0;
  float B = 0.0;
  float C = 0.0;
  float D = 0.0;

  float u = 0;
  float x = 0;
};


class PID final
{
public:
  PID(){}
  void init(float ts, float N, float sat, bool bumpless);
  void setup(float kp, float ki, float kd);
  void reset();
  void reset(float xi, float xd);
  void input(float e);
  void step();
  float output();
  float evolve(float e);
  void show(int i, racs_services__msg__Debug &debug_msg);

private:
  float apply_saturation(float x);

  float ts = 0.0;
  float N = 0.0;
  float sat = 0.0;
  bool bumpless = false;

  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;
  
  float e = 0.0;

  float u1 = 0.0;
  float u2 = 0.0;
  float u3 = 0.0;

  float u = 0.0;

  Integrator integrator;
  Filter derivator;
  Filter lowPassFilter;
};


#endif  // CONTROL_H
