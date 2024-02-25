#include "control.hpp"
#include <cmath>

// Integrator

void Integrator::init(float ts, float sat)
{
  this->ts = ts;
  this->sat = sat;
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
  apply_saturation();
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

void Integrator::apply_saturation()
{
	x = x > +sat ? +sat : x;
	x = x < -sat ? -sat : x;
}


// PID

void PID::init(float ts, float tau, float sat, bool bumpless)
{
  this->ts = ts;
  this->N = N;
  this->sat = sat;
  this->bumpless = bumpless;
  this->integrator = Integrator();
  this->derivator = Filter();
  this->lowPassFilter = Filter();

  integrator.init(ts, 200);
  derivator.init(tau, 1, 0, 1, ts);			// Td*s/(1+s*Td/N)
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

void PID::reset(float u2, float u3)
{
  this-> u2 = u2; //xi
  this-> u3 = u3; //xd
}

void PID::input(float e)
{
  this->e = e;
}

void PID::step()
{
  bool sgn;

  if (abs(e) > 0) {
	  u1 = kp*e;
  	  u2 = ki*integrator.evolve(e);
  	  u3 = kd*derivator.evolve(e);
  	  //sgn = (u1+u2+u3) >= 0;
  	  u = apply_saturation(u1 + u2 + u3 /*+ (2*sgn-1)*9000*/);
  } else
	  u = 0;
}

float PID::output()
{
  return u;
}

float PID::evolve(float e)
{
  input(e);
  step();
  //u = output();

  return u;
}

void PID::show(int i, racs_services__msg__Debug &debug_msg)
{
	debug_msg.data[0] = (float) i;
	debug_msg.data[1] = u1  *100.0/32767.0;
	debug_msg.data[2] = u2  *100.0/32767.0;
	debug_msg.data[3] = u3  *100.0/32767.0;
	debug_msg.data[4] = u   *100.0/32767.0;
	debug_msg.data[5] = e;
}

float PID::apply_saturation(float x)
{
  if(sat > 0)
  {
    x = x > +sat ? +sat : x;
    x = x < -sat ? -sat : x;
  }
  return x;
}


// Filter

void Filter::init(float a1, float a0, float b1, float b0, float ts)
{
  this->a1 = a1;
  this->a0 = a0;
  this->b1 = b1;
  this->b0 = b0;

  A = exp(-(a0/a1)*ts);
  if (a0*a1 != 0) {
	  C = ((a0*b1-a1*b0)/(a0*a1))*(exp((-a0/a1)*ts)-1);
  }
  else
  {
	  C = 0;
  }
  B = 1;
  if (a1 != 0) {
	  D = b1/a1;
  } else
  {
	  D = 0;
  }

  /*A = -(-2*a1+a0*ts)/(2*a1+a0*ts);
  B = 1;
  C = ((2*b1+b0*ts)/(2*a1+a0*ts)) * ((-2*a1+a0*ts)/(2*a1+a0*ts) + (-2*b1+b0*ts)/(2*b1+b0*ts));
  D = ((2*b1+b0*ts)/(2*a1+a0*ts));*/
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
  return C*x + D*u;
}

float Filter::evolve(float u)
{
  float y;
  
  input(u);
  y = output();
  step();

  return y;
}

