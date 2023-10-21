#include "control.hpp"


// Integrator

void Integrator::init(float ts)
{
  this->ts = ts;
}

void Integrator::reset()
{
  reset(0.0);
}

void Integrator::reset(float x)
{
  this->x = x;
}

void Integrator::input(float u)
{
  this->u = u;
}

void Integrator::step()
{
  x = x + ts*u;
}

float Integrator::output()
{
  return x;
}

float Integrator::evolve(float u)
{ 
  float y;

  input(u);
  y = output();
  step();

  return y;
}


// PID

void PID::init(float ts, float pole, float sat, bool bumpless)
{
  this->ts = ts;
  this->pole = pole;
  this->sat = sat;
  this->bumpless = bumpless;

  if(pole > 0)
  {
    A = exp(-pole*ts);
    B = (1-A)/pole;
    C = -pole*pole;
    D = pole;
  }
  else
  {
    A = 0;
    B = 1;
    C = -1/ts;
    D = 1/ts;
  }

  apply_saturation();
}

void PID::setup(float kp, float ki, float kd)
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void PID::reset()
{
  reset(0.0, 0.0);
}

void PID::reset(float xi, float xd)
{
  this-> xi = xi;
  this-> xd = xd;

  apply_saturation();
}

void PID::input(float e)
{
  this->e = e;
}

void PID::step()
{  
  xi = xi + (bumpless ? ki*ts*e : ts*e);
  xd = A*xd + (bumpless ? kd*B*e : B*e);

  apply_saturation();
}

float PID::output()
{
  float u;

  if(bumpless) u = (kp + kd*D) * e + xi + C*xd;
  else u = (kp + kd*D) * e + ki*xi + kd*C*xd;

  return u;
}

float PID::evolve(float e)
{ 
  float u;
  
  input(e);
  u = output();
  step();

  return u;
}

void PID::apply_saturation()
{
  if(sat > 0)
  {
    xi = xi > +sat ? +sat : xi;
    xi = xi < -sat ? -sat : xi;
  }
}


// Filter

void Filter::init(float tau, float ts)
{
  A = exp(-ts/tau);
  B = (1-A)*tau;
  C = 1/tau;
}

void Filter::reset()
{
  reset(0.0);
}

void Filter::reset(float x)
{
  this->x = x;
}

void Filter::input(float u)
{
  this->u = u;
}

void Filter::step()
{
  x = A*x + B*u;
}

float Filter::output()
{
  return C*x;
}

float Filter::evolve(float u)
{
  float y;
  
  input(u);
  y = output();
  step();

  return y;
}

